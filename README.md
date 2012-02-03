# Gavlab ATRV Vehicle Interface

## Documentation

REPLACE ME WITH LINK TO DOCS!

## Dependencies

* CMake, for the build system: http://www.cmake.org/
* MDC2250, for the interface to the motorcontrollers: https://github.com/GAVLab/mdc2250
** Serial, for serial rs-232 interface: https://github.com/wjwwood/serial
** Boost, for threading, function pointers, and shared_ptr's: http://www.boost.org/

## Installation

Get the source:

    git clone https://github.com/GAVLab/gavlab_atrv
    cd mdc2250

Compile the code:

    make

Or run cmake youself:

    mkdir build && cd build
    cmake ..
    make

Install the code (UNIX):

    make
    sudo make install

Uninstall the code (UNIX):

    make
    sudo make uninstall

Run the test (Requires GTest):

    make test

Build the documentation:

    make doc

## License

The BSD License

Copyright (c) 2011 William Woodall

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
