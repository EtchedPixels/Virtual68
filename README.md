# Virtual68
A simple 68K virtual platform for FUZIX bring up and similar purposes

We emulate a very simple platform with a simple serial interface, IDE hard
disc, timer, and some optional MMU, low memory trap and banking features
reflecting those found on some real boards.

To run

make

./makeprom

makedisk 1 disk.img	# makedisk is in the IDE repository

./makehello

./v68


At the moment there is a very simplistic boot "rom" which provides minimal
services via a jump table, and a simple command prompt (b boot r reset)

There are two reasons for Virtual68 existing

1. It provides an extremely simple convenient bring up platform for FUZIX on a
68000

2. It contains a software implementation of the proposed ridiculously simple
MMU that never quite made the Atari ST series(*). One of the goals is to boot
FUZIX using that MMU and see if it actualy works out, before trying to do the
same on an FPGA 68K.

(*)http://www.dadhacker.com/blog/?p=1383


Based upon the Mushashi 680x0 emulator code base which is

LICENSE AND COPYRIGHT:
---------------------

Copyright © 1998-2001 Karl Stenerud

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



AVAILABILITY:
------------
The latest version of this code can be obtained at:
https://github.com/kstenerud/Musashi
