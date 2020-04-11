*****************************************************************************
*** Files Organization                                                    ***
*****************************************************************************

--{root}                  - Root directory.
  +--readme.txt           - This file.
  +--todo.txt             - Current plan (development/unstable versions only).
  +--license.txt          - GPL license text.
  +--demos/               - Demo projects.
  +--docs/                - Documentation.
  +--mz_libs/             - PLIB MZ files.
     +--verinfo.txt       - Version number info.
     +--include/          - Low level Hardware Abstraction Layer.
        +--plib_mz.h      - Low level drivers main header.
        +--peripheral_mz/ - Low level drivers implementations.


*****************************************************************************
*** Releases                                                              ***
*****************************************************************************
*** 0.1.4 ***
-Added low-level driver for CAN
*** 0.1.3 ***
-Added low-level driver for RTCC
*** 0.1.2 ***
-Improve NVM function for write and erase.
-Add optional aditional elements for synchronization with external NVM drivers.

*** 0.1.1 ***
-Add aditional definitions of the names of functions.
-Change of the functions names in "nvm.h".
-Change of the code in "uart.h" of the function "UARTSetDataRate". 
Now is with auto config of BRG bit. 
-Add new functions in "osc.h". Improve the code.
-Minor changes to other files.
-Chanve wersionname in "verinfo.txt".

*** 0.1.0 ***
- First release
