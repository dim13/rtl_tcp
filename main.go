// Command rtl_tcp serves RTL-SDR I/Q samples over the rtl_tcp protocol.
package main

import (
	"encoding/binary"
	"errors"
	"flag"
	"fmt"
	"io"
	"log"
	"net"
	"sync"

	"github.com/dim13/r82xx"
	"github.com/dim13/r82xx/tuner"
)

const (
	bufSize = 262_144 // 16 * 32 * 512
	bufs    = 64      // ~3.4 s at 2.4 MS/s before samples are dropped
)

// librtlsdr tuner type reported in the header.
// ponytail: r82xx.Device does not expose R820T vs R828D; both share
// the gain table, which is all clients use this for.
const tunerR820T = 5

type command byte

// rtl_tcp commands.
const (
	_                 command = iota
	cmdFrequency              // Hz
	cmdSampleRate             // Hz
	cmdGainMode               // 0 auto, 1 manual
	cmdGain                   // tenths of dB
	cmdFreqCorrection         // ppm
	cmdIFGain                 // stage<<16 | tenths of dB
	cmdTestMode               // 0/1
	cmdAGCMode                // 0/1
	cmdDirectSampling         // 0 off, 1 I, 2 Q
	cmdOffsetTuning           // 0/1
	cmdRTLXtal                // Hz
	cmdTunerXtal              // Hz
	cmdGainByIndex            // index into gain table
	cmdBiasTee                // 0/1
)

type server struct {
	dev  *r82xx.Device
	freq float64
}

func (s *server) command(c command, arg uint32) error {
	switch c {
	case cmdFrequency:
		s.freq = float64(arg)
		return s.dev.SetFrequency(s.freq)
	case cmdSampleRate:
		return setSampleRate(s.dev, arg)
	case cmdGainMode:
		if arg == 0 {
			return s.dev.SetGainAuto()
		}
		return s.dev.SetGainManual()
	case cmdGain:
		return s.dev.SetGain(int(int32(arg)))
	case cmdFreqCorrection:
		if err := s.dev.SetPPM(float64(int32(arg))); err != nil {
			return err
		}
		return s.dev.SetFrequency(s.freq)
	case cmdGainByIndex:
		if int(arg) >= len(tuner.GainSteps) {
			return fmt.Errorf("gain index %d out of range", arg)
		}
		return s.dev.SetGain(tuner.GainSteps[arg])
	case cmdAGCMode:
		return s.dev.SetAGC(arg != 0)
	case cmdTestMode, cmdDirectSampling, cmdOffsetTuning, cmdBiasTee:
		// Off is the power-on state; clients send it on connect.
		if arg == 0 {
			return nil
		}
		return errors.ErrUnsupported
	default:
		return errors.ErrUnsupported
	}
}

func (s *server) serve(conn net.Conn) error {
	defer conn.Close()
	if err := s.dev.ResetBuffer(); err != nil {
		return err
	}
	hdr := make([]byte, 12)
	copy(hdr, "RTL0")
	binary.BigEndian.PutUint32(hdr[4:], tunerR820T)
	binary.BigEndian.PutUint32(hdr[8:], uint32(len(tuner.GainSteps)))
	if _, err := conn.Write(hdr); err != nil {
		return err
	}

	var wg sync.WaitGroup
	done := make(chan struct{})
	samples := make(chan []byte, bufs)
	// Decouple USB from TCP so a slow client drops samples instead of
	// stalling the device FIFO.
	wg.Go(func() {
		defer close(samples)
		for {
			select {
			case <-done:
				return
			default:
			}
			buf := make([]byte, bufSize)
			n, err := s.dev.Read(buf)
			if err != nil {
				log.Println("read:", err)
				conn.Close()
				return
			}
			select {
			case samples <- buf[:n]:
			default:
				log.Println("client too slow, dropping samples")
			}
		}
	})
	wg.Go(func() {
		for b := range samples {
			if _, err := conn.Write(b); err != nil {
				conn.Close()
			}
		}
	})

	var err error
	cmd := make([]byte, 5)
	for {
		if _, err = io.ReadFull(conn, cmd); err != nil {
			break
		}
		c, arg := command(cmd[0]), binary.BigEndian.Uint32(cmd[1:])
		if err := s.command(c, arg); err != nil {
			log.Printf("command %#x %d: %v", c, arg, err)
		}
	}
	close(done)
	conn.Close()
	wg.Wait()
	if errors.Is(err, io.EOF) || errors.Is(err, net.ErrClosed) {
		return nil
	}
	return err
}

// setSampleRate also sets the tuner filter to match, as librtlsdr does.
func setSampleRate(dev *r82xx.Device, hz uint32) error {
	if _, err := dev.SetSampleRate(hz); err != nil {
		return err
	}
	return dev.SetBandwidth(0, float64(hz))
}

func main() {
	var (
		addr = flag.String("a", "127.0.0.1:1234", "listen address")
		freq = flag.Float64("f", 100e6, "frequency in Hz")
		rate = flag.Uint("s", 2_048_000, "sample rate in Hz")
		gain = flag.Float64("g", 0, "gain in dB, 0 for auto")
		ppm  = flag.Int("P", 0, "frequency correction in ppm")
	)
	flag.Parse()

	u, err := openUSB()
	if err != nil {
		log.Fatal("usb: ", err)
	}
	defer u.Close()
	dev, err := r82xx.Open(u)
	if err != nil {
		log.Fatal("open: ", err)
	}
	defer dev.Close()

	if *ppm != 0 {
		if err := dev.SetPPM(float64(*ppm)); err != nil {
			log.Fatal("ppm: ", err)
		}
	}
	if err := setSampleRate(dev, uint32(*rate)); err != nil {
		log.Fatal("sample rate: ", err)
	}
	if err := dev.SetFrequency(*freq); err != nil {
		log.Fatal("frequency: ", err)
	}
	if *gain == 0 {
		err = dev.SetGainAuto()
	} else {
		err = dev.SetGain(int(*gain * 10))
	}
	if err != nil {
		log.Fatal("gain: ", err)
	}

	l, err := net.Listen("tcp", *addr)
	if err != nil {
		log.Fatal("listen: ", err)
	}
	log.Println("listening on", l.Addr())
	s := &server{dev: dev, freq: *freq}
	// rtl_tcp serves one client at a time.
	for {
		conn, err := l.Accept()
		if err != nil {
			log.Fatal("accept: ", err)
		}
		log.Println("client connected")
		if err := s.serve(conn); err != nil {
			log.Println("serve:", err)
		}
		log.Println("client disconnected")
	}
}
