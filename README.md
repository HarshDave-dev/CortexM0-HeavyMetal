# CortexM0-HeavyMetal

This is a bare metal code offereing minimal features such as clock(HSI48)/system bus(APB, AHB) configurations, system tick and GPIO configuration.

Once the startup sequence which involves setting up of the vector table, initializing .bss, stack, head, data secction is done, We configure clock source, its prescalers, APB, AHB bus clock configuration and finally GPIO configuration.

After that, we end up in an infinit loop, along with having a LED toggling on system tick interrupt.

Its pretty much what this code does.

# Compiling
First, you'll need the arm-none-eabi toolchain, which I won't cover installing here.
After having that, you'll need to download these sources. Then, the following commands should suffice:

$ cd Build                                                                                                                                                                          
$ make all
