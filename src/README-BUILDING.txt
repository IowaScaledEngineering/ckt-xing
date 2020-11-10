**************************
Cross-Compiler Setup

If you don't have the ARM GCC cross-compiler installed...

1) Download the ARM GCC toolchain - currently at:
https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

2) Untar the output in /opt (or whever you prefer)

3) Create a symlink /opt/gcc-arm to the full directory.  This allows easily
switching compiler versions in the future

4) In ~/.profile, add /opt/gcc-arm/bin on the front of your path variable
PATH="/opt/gcc-arm/bin:$PATH"

That should be it.  Should just be able to type "make" in the src directory
after that.


****************************
Flashing the Part
arm-none-eabi-gdb -ex 'target extended-remote /dev/ttyACM2' -ex 'monitor swdp_scan' -ex 'attach 1' -ex 'load' ckt-xing.elf

Once it's loaded, type 'run' at the gdb prompt and so on
