# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Summary of set up
* Configuration
* Dependencies
* Database configuration
* How to run tests
* Deployment instructions

### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

# Debug #
## Debug With Eclipse ##
OpenRTOS Viewer
Queue table
Semaphores are taken when "Current length" is 0.
Semaphores are given when "Current length" is 1.

## Debug Core Dumps ##
1.1	Reading the symbols:
To do a disassembly.
 gna@ubuntu:~/gnacode/controller-sw$ arm-none-eabi-objdump -D controller > dis.txt

 arm-none-eabi-readelf –ls controller

Searching a function by address:
gna@ubuntu:~/gnacode/controller-sw$ arm-none-eabi-nm -n controller | grep 0800


1.2	GDB
gna@ubuntu:~/gnacode/controller-sw$ arm-none-eabi-gdb controller
(gdb) target remote localhost:3333

(gdb) monitor reset halt

The current Task Control Block – e.g. the name of the currently running task.
pxCurrentTCB
(gdb) print *pxCurrentTCB

Memory dump:
(gdb) x/100x 0x2000fd00

(gdb) info reg

Run program:
(gdb) c
Use “<ctrl>-c” to break

    hardFaultSP = (0x20000000)
    r0    = (hardFaultSP + 96)	; 0x60
    r1    = (hardFaultSP + 92)	; 0x5c
    r2    = (hardFaultSP + 88)	; 0x58
    r3    = (hardFaultSP + 84)	; 0x54
    r4    = (hardFaultSP + 80)	; 0x50
    r5    = (hardFaultSP + 76)	; 0x4c
    r6    = (hardFaultSP + 72)	; 0x48
    r7    = (hardFaultSP + 68)	; 0x44
    r8    = (hardFaultSP + 64)	; 0x40
    r9    = (hardFaultSP + 60)	; 0x3c
    r10   = (hardFaultSP + 56)	; 0x38
    r11   = (hardFaultSP + 52)	; 0x34
    r12   = (hardFaultSP + 48)	; 0x30
    lr    = (hardFaultSP + 44)	; 0x2c
    pc    = (hardFaultSP + 40)	; 0x28
    psr   = (hardFaultSP + 36)	; 0x24
    _CFSR = (hardFaultSP + 32)	; 0x20
    _HFSR = (hardFaultSP + 28)	; 0x1c
    _DFSR = (hardFaultSP + 24)	; 0x18
    _AFSR = (hardFaultSP + 20)	; 0x14
    _MMAR = (hardFaultSP + 16)	; 0x10
    _BFAR = (hardFaultSP + 12)	; 0x0c
 
So to find the value of r7 read the contents of 0x20000000 and add 68 this is the address to look in for the value of r7.
(gdb) x/1x 0x2000ff4c+68

  _CFSR	Configurable Fault Status Register
Consists of MMSR, BFSR and UFSR
                                                                                  
  _HFSR	Hard Fault Status Register
  
  _DFSR	Debug Fault Status Register
  _AFSR 	Auxiliary Fault Status Register
  
  _MMAR	Read the Fault Address Registers. These may not contain valid values.
Check BFARVALID/MMARVALID to see if they are valid values
MemManage Fault Address Register

  _BFAR	Bus Fault Address Register


Memory usage – the heap has got these two variables of interest:
/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
    (gdb) print xFreeBytesRemaining
    (gdb) print xMinimumEverFreeBytesRemaining
    (gdb) x 0x20000000	-- > PC for hard fault handler
     x /nnw 0x2000xxxx 	-- > dump nn words from addr 0x2000xxxx 

1.3	Analysing core dumps

In case of late attach (e.g. attach after assertion or hard fault ):
gna@ubuntu:~/gnacode/controller-sw$ arm-none-eabi-gdb controller
    (gdb) target remote localhost:3333 
    (gdb) monitor  halt
    (gdb) dump binary memory <file-name>.bin 0x20000000 0x20010000
    (gdb) print *pxCurrentTCB
    (gdb) x 0x20000000	-- > SP for hard fault handler
    (gdb) print pc 	-- > to find PC at time of HardFault
    (gdb) print *file 	-- > to find assertion
    (gdb) print line  	-- > to find assertion
    (gdb) info reg


    (gdb) load 
    (gdb) restore gdb_dump_18-03-2015 binary 0x20000000 



Example session:

 gna@ubuntu:~/gnacode/controller-sw$ arm-none-eabi-gdb controller
 GNU gdb (GNU Tools for ARM Embedded Processors) 7.4.1.20130613-cvs
 Copyright (C) 2012 Free Software Foundation, Inc.
 License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
 This is free software: you are free to change and redistribute it.
 There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
 and "show warranty" for details.
 This GDB was configured as "--host=i686-linux-gnu --target=arm-none-eabi".
 For bug reporting instructions, please see:
 <http://www.gnu.org/software/gdb/bugs/>...
 Reading symbols from /home/gna/gnacode/controller-sw/controller...done.
 (gdb) target remote localhost:3333
 Remote debugging using localhost:3333
 0x00000000 in ?? ()
 (gdb) monitor halt
 target was in unknown state when halt was requested
 target state: halted
 target halted due to debug-request, current mode: Handler HardFault
 xPSR: 0x61000003 pc: 0x08000d6a msp: 0x2000ff4c
 (gdb) x 0x20000000
 0x20000000:	0x2000ff4c
 (gdb)


 
2 Debug session record

 (gdb) x 0x20000000
 0x20000000:	0x2000ff4c	-> 

 (gdb) x /10bs 0x20000000
 0x20000000:	 "L\377"
 0x20000003:	 " \002"
 0x20000006:	 ""
 0x20000007:	 ""
 0x20000008:	 "2.000.000"
 0x20000012:	 ""
 0x20000013:	 ""
 0x20000014:	 "2015.05.19-20:03:11"
 0x20000028:	 "bdc4cd3-dirty"
 0x20000036:	 ""

 (gdb) x /30wx 0x2000ff4c
 0x2000ff4c:	0x080014d1	0x200031c8	0x40004400	0xe000ed38
 0x2000ff5c:	0xe000ed34	0x00000000	0x00000008	0x40000000
 0x2000ff6c:	0x00000400	0x81000000	0x08012e44	0x08012c83
 0x2000ff7c:	0x01010101	0xa5a5a5a5	0xa5a5a5a5	0xa5a5a5a5
 0x2000ff8c:	0xa5a5a5a5	0x2000ff4c	0xa5a5a5a5	0xa5a5a5a5
 0x2000ff9c:	0xa5a5a5a5	0x00000000	0x200067a0	0x00000000
 0x2000ffac:	0x200067a0	0xa5a5a5a5	0x2000ffb8	0xa5a5a5a5
 0x2000ffbc:	0xa5a5a5a5	0xa5a5a5a5 


### Who do I talk to? ###

* Repo owner or admin
* Other community or team contact