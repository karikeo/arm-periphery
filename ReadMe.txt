ReadMe.txt for BSP for STM32F10 start project.

This project was built for the IAR Workbench for ARM V6.40

Supported hardware:
===================
The sample project for the ST STM32F103 CPU
is prepared to run on a ST MB525 eval board,
but may be used on other target hardware as well.

Configurations
==============
- Debug:
  This configuration is prepared for download into
  internal Flash using J-Link and CSpy.
  An embOS debug and profiling library is used.

- Release:
  This configuration is prepared to build a "release"
  output which can be downloaded into FLASH
  using CSPy and IAR Flashloader.

Using different target hardware may require modifications.
