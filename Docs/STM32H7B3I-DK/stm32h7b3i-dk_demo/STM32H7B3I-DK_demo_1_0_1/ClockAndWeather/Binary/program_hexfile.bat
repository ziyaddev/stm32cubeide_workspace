:: STM32CubeProgrammer Utility flash script
@ECHO OFF
@setlocal

:: Current Directory
@SET CUR_DIR=%CD%

:: Board defs
@SET BOARD_ID=0
@SET CHIP_NAME=STM32H7B3I
@SET MAIN_BOARD=DK
@SET EXT_LOADER=MX25LM51245G_%CHIP_NAME%-DISCO

:: Demo Name
@SET DEMO_NAME=ClockAndWeather
@SET DEMO_TAG=_
@SET DEMO_VER=V1.0.1

:: Demo defs
@SET HEX_FILE=%DEMO_NAME%-%CHIP_NAME%-%MAIN_BOARD%%DEMO_TAG%%DEMO_VER%.hex

TITLE STM32CubeProgrammer Utility for %CHIP_NAME%%MAIN_BOARD%

@SET STM32_PROGRAMMER_PATH="%ProgramFiles(x86)%\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin"
@IF NOT EXIST %STM32_PROGRAMMER_PATH% @SET STM32_PROGRAMMER_PATH="%ProgramW6432%\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin"
@IF NOT EXIST %STM32_PROGRAMMER_PATH% @SET STM32_PROGRAMMER_PATH="%ProgramFiles%\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin"
@IF NOT EXIST %STM32_PROGRAMMER_PATH% goto goError

@SET STM32_EXT_FLASH_LOADER=%STM32_PROGRAMMER_PATH%\ExternalLoader\%EXT_LOADER%.stldr

:: Do some checks
@IF NOT EXIST %HEX_FILE% @ECHO %HEX_FILE% Does not exist !! && GOTO goError
@IF NOT EXIST %STM32_PROGRAMMER_PATH% @ECHO %STM32_PROGRAMMER_PATH% Does not exist !! && GOTO goError
@IF NOT EXIST %STM32_EXT_FLASH_LOADER% @ECHO %STM32_EXT_FLASH_LOADER% Does not exist !! && GOTO goError

:: Add ST-link utility to the PATH
@SET PATH=%STM32_PROGRAMMER_PATH%;%PATH%

:StartProg
@ECHO.
@ECHO =============================================
@ECHO Programming %HEX_FILE% on board id %BOARD_ID%
@ECHO =============================================
@ECHO.
STM32_Programmer_CLI.exe -c port=SWD index=%BOARD_ID% reset=HWrst -e all
STM32_Programmer_CLI.exe -c port=SWD index=%BOARD_ID% reset=HWrst -el %STM32_EXT_FLASH_LOADER% -d %HEX_FILE% -HardRst
@IF NOT ERRORLEVEL 0 (
  @GOTO goError
)

@GOTO goOut

:goError
@SET RETERROR=%ERRORLEVEL%
@COLOR 0C
@ECHO.
@ECHO Failure Reason Given is %RETERROR%
@PAUSE
@COLOR 07
@EXIT /b %RETERROR%

:goOut
