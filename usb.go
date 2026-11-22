package main

import (
	"context"
	"errors"
	"time"

	"github.com/google/gousb"
)

const (
	vendorID  = 0x0bda
	productID = 0x2838

	controlIn  = gousb.ControlIn | gousb.ControlVendor | gousb.ControlDevice
	controlOut = gousb.ControlOut | gousb.ControlVendor | gousb.ControlDevice
)

var errNotFound = errors.New("device not found")

// usb implements wire.Transport on top of gousb.
type usb struct {
	ctx  *gousb.Context
	dev  *gousb.Device
	done func()
	in   *gousb.InEndpoint
}

func openUSB() (*usb, error) {
	ctx := gousb.NewContext()
	dev, err := ctx.OpenDeviceWithVIDPID(vendorID, productID)
	if err != nil {
		ctx.Close()
		return nil, err
	}
	if dev == nil {
		ctx.Close()
		return nil, errNotFound
	}
	dev.ControlTimeout = 300 * time.Millisecond
	dev.SetAutoDetach(true)
	intf, done, err := dev.DefaultInterface()
	if err != nil {
		dev.Close()
		ctx.Close()
		return nil, err
	}
	in, err := intf.InEndpoint(1)
	if err != nil {
		done()
		dev.Close()
		ctx.Close()
		return nil, err
	}
	u := &usb{
		ctx:  ctx,
		dev:  dev,
		done: done,
		in:   in,
	}
	return u, nil
}

func (u *usb) ControlIn(wValue, wIndex uint16, data []byte) (int, error) {
	return u.dev.Control(controlIn, 0, wValue, wIndex, data)
}

func (u *usb) ControlOut(wValue, wIndex uint16, data []byte) error {
	_, err := u.dev.Control(controlOut, 0, wValue, wIndex, data)
	return err
}

func (u *usb) BulkIn(data []byte) (int, error) {
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	return u.in.ReadContext(ctx, data)
}

func (u *usb) Strings() (string, string, error) {
	m, err := u.dev.Manufacturer()
	if err != nil {
		return "", "", err
	}
	p, err := u.dev.Product()
	if err != nil {
		return "", "", err
	}
	return m, p, nil
}

func (u *usb) Close() error {
	u.done()
	err := u.dev.Close()
	if cerr := u.ctx.Close(); err == nil {
		err = cerr
	}
	return err
}
