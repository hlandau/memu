memu
====

This is an emulator/simulator for the ARMv8-M ISA, which I developed in mid
2021. It was designed for high accuracy by exactly matching, where possible,
the psuedocode in the ARMv8-M ISA manual. It is a modular C++17 library which
can be embedded in your own program and used with your own memory subsystem.
The code has also been designed to be highly readable and written for clarity,
not performance, and to have the syntactic structure of the code match the ISA
manual as much as possible.

The simulator is written in a single C++17 file for ease of inclusion in your
project.

It is a template class, which allows you to instantiate it for specific
interfaces, allowing the compiler to inline implementations (e.g. if you use
final classes), enhancing performance if you wish.

This *will* still be buggy because the test suite hasn't been completed yet; I
don't currently have time to finish it. It's also oriented to simulating
ARMv8-M and not ARMv7-M; since ARMv7-M has e.g. different MPU registers, it
won't correspond exactly to ARMv7-M.

[The TODO list can be found here.](https://github.com/hlandau/memu/blob/master/emu2.cc#L56)

The script `genundef` can be used to generate a file which
pushes/pops/undefines all macros defined in the C++17 file, if you don't want
these macros to pollute your project.

If you want to use it, I'd suggest emailing me to get some pointers.
In any case, it's licenced under GPLv3 or, at your option, any later version.

    © 2021 Hugo Landau <hlandau@devever.net>

