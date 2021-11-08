# STM32H7B3I

## EWARM Support

EWARM does not come with support for the STM32H7B3I, therefore EWARM support files are stored inside
`3rdparty/EWARMv8_STM32H7Ax-7Bx_Support_V2.3.zip`.

Installing `EWARMv8_STM32H7Ax-7Bx_Support_V2.3.exe` will add the following:
- RPNs without SMPS:STM32H7A/B3xx
- RPNs with SMPS:STM32H7A/B3xx-Q
- Adding support for new RPNs value line: STM32HB0xx
- STM32H7B3I-EVAL dedicated connection with OSPI external loader support (revB)
- STM32H7B3I-EVAL dedicated connection with FMC NOR external loader support
- STM32H7B3I_DISCO dedicated connection with OSPI external loader support
- STM32H7B0x_DISCO dedicated connection with OSPI external loader support
- STM32H7B0x_EVAL dedicated connection with OSPI external loader support
- Automatic STM32H7A/B flash algorithm selection
- SVD file for STM32H7A/B

### Installation Instructions

1. Extract the contents of `3rdparty/EWARMv8_STM32H7Ax-7Bx_Support_V2.3` to a temporary folder.
2. Copy `EWARMv8_STM32H7Ax-7Bx_Support_V2.3.exe` to your EWARM installation folder, typically `C:\Program Files\IAR Systems\Embedded Workbench`.
3. Then run `EWARMv8_STM32H7Ax-7Bx_Support_V2.3.exe` as administrator.


### Uninstallation instructions
You can remove it by running `Uninstall_Patch.bat` as administrator.
