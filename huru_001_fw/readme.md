# HURU PT10 FW

## Setup

First download the following 
- VS Code (Note: Not Visual studio code) https://code.visualstudio.com/download
// - GNU make (via choclatey) https://stackoverflow.com/a/32127632
- mingw32-make via MinGW https://www.mingw-w64.org/
- GNU core utils for Windows https://sourceforge.net/projects/gnuwin32/files/coreutils/5.3.0/coreutils-5.3.0.exe/download?use_mirror=netix&download=
- arm embedded toolchain https://developer.arm.com/downloads/-/gnu-rm
- nRF5_SDK_17.1.0_ddde560 https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs
- NRF connect for desktop https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop/Download#infotabs
- nrfjprog https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download
- nrfutil (not needed for this but v-useful)


1. open the code folder within VSCode. Edit the makefile to point to your download locations for the SDK
2. Edit the Lauch.json file to point to the .SVD file location and your ARM toolchain path
5. add the make install location to your system path
6. edit the makefile.windows to point to your arm embedded toolchain
3. to check the enviroment is setup run the "make" command in the terminal  
4. open nrfconnect for desktop and install the programmer application
7. edit nRF5_SDK_17.1.0_ddde560\external\fatfs\src\ffconf.h line 50 "_USE_EXPAND" to equal 1
8. c_cpp_config
9. cortex-debug install
10.

makefile config
## Building the code

run "make" or "mingw32-make" to build the code
"make flash" to build and flash the code via j-link
make package to create a dfu image
make release to complie the app + bootloader +SD
make erase to clean the current device

## Key Layers
* core: upper layers - behaviours and device independent code
* drivers: lower layers - device specific drivers and SOUP (middleware)
## Key Files
* main.c - setup for bluetooth stack, initalisation of hardware
* PT10.c - device specific implementation
* huru_ser.c - setup for the eeg bluetooth charatristics. collects incomming messages and puts these into a buffer to be serviced by the bluetooth.c handler
* bluetooth.c - handles bluetooth messages.
* data.c - compilation, storage and transmission of epoch data
* power.c - charging states & battery voltage
* board_config.h - set the target device & functional mode
### Hardware files
* AFE4060.c - EEG Analog Front End driver
* LIS2DTW12.c - Accelerometer driver
* MT25QL256ABA1EW7.c - Flash memory driver
* RV3028.c - RTC driver

## Bluetooth 

### services
The following services and their characteristics are implemented:

EEG service
- eeg command, 5 byte,
- eeg data, 232 byte, 

Lead detect
- Lead on/ off , 1 byte, bool
- Lead quality (impedance), 1 byte, uint8 0-100


### messages

All messages are 5 bytes long formed of a byte array.
The following messages are implimented:
(if bytes are not present, then they are don't care)

| name | message | details | reponse | implimented |
|------|---------|---------|---------|-------------|
| remaining bytes | 0x01 | reply with the remaining bytes of data remaining | n |
| send next data to app | 0x02 | send the next data | none* | y |
| send previous data to app | 0x03 | resend the previous data | none* | n |
| send next data to script | 0x04 | resend the next data | none* | n |
| set device ID | 0x05 xx xx xx xx | set the device ID (max 2000) | 0x05 xx xx xx xx | y |
| get device ID | 0x06 | get the device ID | 0x06 xx xx xx xx | y |
| set time | 0x07 xx xx xx xx | set the time  | 0x07 xx xx xx xx | y |
| get time | 0x08 | resend the previous data | 0x08 xx xx xx xx | y |
| Enter DFU | 0xFE | put into DFU mode, battery voltage needs to be above 15% | none* | y |
| Shutdown | 0xFF | put into sleep mode | none* | y |

*will respond on the eeg data charateristic

### Building a device
- Functional test the new board on a programming fixture.
-- make flash_all
-- debug and observe output of init messsges
- Assemble connector to main board
- Place cells on bottom board
- Place assembled connector and main board onto bottom board
- Solder connector to bottom board
- Position and Solder cells from the non-connector side
- Solder cells on the connector side
- Note serial number, and place assembled unit on dock
- Scan and connect with (eg. Huru Live Viewer)
- Enter serial number 05 00 00 00 ab
- Reconnect
- Set RTC
- Enable EEG Debug (streaming) mode
- place on signal generator attenuator pads
- check SNR in passive and active modes
- Shut unit down (ff 00 00 00 00)


