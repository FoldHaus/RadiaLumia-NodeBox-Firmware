# FoldHaus DMX Node Firmware

This firmware is for running on an AVR ATmega328p that is in each node of the sphere.

## Features

This software does 3 things:
 - Read DMX512 data from Pixlite output
 - Generate A/B/EN motor signals from DMX data
 - Turn on and off LED driver from DMX data

## Prerequisites - Help develop this

To help develop this software, you'll need to install [PlatformIO](https://platformio.org).
Think of PlatformIO as a modern take on the Arduino "IDE".
It's all the same code, just with easier to use text editor with many more features including managing any needed libraries.

If you already have the [Atom](https://atom.io) editor, just install the `platformio-ide` package and it should take care of the rest.

With PlatformIO setup, you should be able to open this directory and it should give you a "Build"/"Upload" button that is basically ready to go.

There might be some minor changes necessary to specify the correct serial port but PIO is pretty good about autodetecting.
