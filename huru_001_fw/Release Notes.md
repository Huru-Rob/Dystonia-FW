# Release Notes

## 1.18
- HURU Protocol Version 0x0103 now supported, but not turned on yet to maintain compatibility with the current build of the app
    - Temperature now recorded once per epoch
    - Battery level now recorded once per epoch
    - These items are not transmitted to the app yet, though.
- 2ms delay removed from reset sequence of AFE to speed up deep sleep connection impedance measurement
- Device ID change to CLIC-MM-YY-XXXXX supported, but conditionally compiled back out to maintain compatibility with the app
- Sticky Pad wake-up from Deep Sleep replaced with Connection Impedance detection
- On transition from RECORDING to IDLE with Flash Full, afe and acc no longer turned off, because they would not be turned back on again if the flash empties enough to begin recording again

## 1.17
- HFCLK no longer requested explicitly. This allows the power management system to release the HFCLK during wait for a saving of 200uA
- on transitions to and from ON_DOCK to IDLE and RECORD, power handling fixed such that AFE, MAG and ACC are always turned off on return to ON_DOCK
- de-initialising of SPI, ADC and I2C added on entry to SLEEP mode for power saving.
- Initialising of byte_count and last_byte_count forced to be equal at the start of every Serial Rx state
- Serial Rx process stopped and reset in the event of Rx CRC error 

## 1.16
- Epoch recording start-up mechanism changed to support time-sync
    - hw_rtc now used to generate an update interrupt synchronised to the one second update clock
    - on receipt of the interrupt, afe fifo is cleared and recording begins
    - periodic update interface added to hw_rtc
        - periodic update disabled during hw_rtc initialisation in case of crash/hang whilst enabled
    - 5ms delay removed from afe_pause()
    - afe_clear_fifo() method added to clear fifo just before starting the first epoch
    - epoch timestamp managed locally after initial request to hw_rtc at the start of recording
    - timestamp used in epoch synthesis now managed locally after initial hw_rtc request at the start of synthesis
- Transition from ON_DOCK to IDLE now allowed if UART connection is present to allow recording of time-sync epoch on request from multi-dock
- Init value of AC amplitude (-1) forced to 0 to prevent negative values emerging from impedance system 
- Offset correction added to reporting of impedance via characteristic, and via command
- Storage Reason added to update of internal Flash values (device ID and deep sleep flag)
    - Command added to retrieve the reset reason
    - can be used now to investigate the reason for loss/resetting of device ID
- PWM peripheral disabled via uninit when LEDs stop. Attempt to save power, but negiligible outcome.
- gpio_init brought forward in power-up sequence to aid debug of start-up timing
- check_for_sticky_pad brought forward in attempt to speed up periodic wake-ups (negligible outcome).
- hal_set_board_for_shutdown() added to entry into deep sleep mode
- tx_abort added to serialStopTx in attempt to save power (suggested workaround for api bug which fails to stop hf clock). Negligible outcome.
- rx_abort and return to idle state added in the event of crc failure during serial rx. 
- serialStopRx and return to idle state moved outside test of rx_done to avoid potential lock-up of serial receive process
- serialStopTx and return to idle state added to serial tx state upon receipt of serial_stop command.
- uart connection dropped and return to idle state in event of command timeout during serial tx state
    - prevents device from hanging in the serial tx state if no command response is prepared.
- serialStopTx and return to idle state added to wait_tx_done state to allow reset of serial state machine in the event that the tx process hangs.
- storage_format() re-instated in the event that storage_init() discovers a corrupted filesystem


## 1.15
- All remaining error codes throughout the whole code-base uniquified
- Epoch storage state machine no longer run in the event of garbage collector encountering an error
- Epoch duration timer magic numbers replaced with call to AFE to get FIFO fill time
- Storage integrity check added (for debug only)
- Flash Block Write now waits for WIP bit to clear before resuming suspended erase operation
- Flash Erase Sector now checks for erase error in the preceding sector erase operation before continuing

## 1.14
- missing flags for suspended erase operation added. 

## 1.13
- Panic state added. Errors encountered while recording now result in panic state being entered.
- Panic information command added. Error code resulting in the panic state can be retrieved while the device is in the panic state.
- All error codes resulting from storage and flash drivers uniquified. 
- Epoch transmission bug fixed whereby the incorrect number of messages could be declared if the epoch size exactly fits an integer number of messages.
    - This is a bug, but is believed never to have been encountered in testing.
- Watchdog initialisation moved until after initialisation of the softdevice, to allow watchdog actually to reset the device.
- DC DC converter enabled when using the softdevice to save ~100uA of operating current.
- Storage system no longer tested on initialisation if the flash device check has already failed
- Garbage collection changed from erasing 32kByte subsectors to erasing 64kByte sectors
    - The MT25 data sheet states that reads from the same sector as a suspended erase operation may return inteterminate data
- Check for correct continuation page headers added to reading from tail file
- 4-byte 32kByte Subsector erase flash command corrected
- Flash initialisation updated
    - Multiple attempts to read ID removed
    - Read of status byte and flag status byte added
    - Error generated if 4-byte address mode not confirmed.
    - Check for 3v device type added
- Check added for flash erase being suspended but not resumed
- Multiple attempts now allowed with delay to confirm suspension of erase operation and readiness of program/erase controller
- Internal Pullup enabled on UART input during serial operation to prevent 50Hz pickup from generating UART framing errors 


## 1.12
- epoch recording, storage and transmission state machine states added to report
- storage blank/integrity check added, unused in release
- errors when opening tail file now reported at warning level
- uart framing errors handled without calling APP_ERROR_CHECK
- used to erased transitions added to first free page calculation
- multiple attempts to retrieve flash id now allowed and reported

## 1.11

- Power-On Self Test results now collated, and made available by a new device command
- New device commands added to retrieve firmware version and hardware version (for use when on the multi-dock)
- Magnetometer forced off on entry to Deep Sleep (previously missing)
- Serial payload type CRC error added to allow valid responses to the app in the presence of CRC errors
- Magnetometer set to Low Power mode to save 125uA
- Phy update removed from BLE Connected callback to avoid collision with request from central to update Phy.
- Initialisation of Flash Erase reduced to erasing one subsector to allow erase command to return immediately, and avoid UART disconnect
- Impedance streaming update timer value fixed
- Offset on reported source impedance caused by on-board series resistances removed to first order.

## 1.10

- UART Connection timeout increased to 250ms to allow for both (Rx and Tx) ports of the half-duplex UART to have a round-trip
- Minor changes to invocations of serialStart()
- Major change to UART tx packet creation for responding to prevent multiple responses with the same packet
- Separation of UART command responses from the asynchronous packet queues to prevent lock-out of command responses
- Epoch SEND_COMPLETE command added for use by Multi-Dock to terminate epoch download 
- Delay of 1s reduced to 10ms at the start of enterShipMode() to prevent UART timeout when sending command
- Flash Erase now defers all actual erasure until processed in the background to allow command to return immediately - preventing UART timeout
- Flash last_write_command bug fixed to correctly log erase commands

## 1.09

- Use of logging overhauled to comply with Nordic intent
- Module-specific logging introduced
- Default log level increaed globally to 4 (DEBUG)
    - Overridden locally per-module to 3 (INFO)
- All NRF_LOG_FLUSH() macros removed to avoid potential interruption of one by another
- NRF_LOG_FLUSH() specifically removed from serial UART handler (in interrupt context) 
- Impedance buffer overrun check improved when fetching connection quality information 
- Return codes from pushing impedance data to the characteristic processed to avoid confusing error logs when Alex doesn't read impedance notifications
- Storage initialisation re-factored to avoid large nesting.
    - Header transition parsing improved to log all possible transitions
- Potential storage lock-up situation removed, whereby a final page containing only a header could have been written.
    - This would hang the read-back process, as the expected next page header would be START, not CONTINUATION
- Subsector erase algorithm improved to erase right up to the start of data, not one subsector short.
- RTC interface initialisation improved
- RTC register dump function added. 

## 1.08

- External pin enabled (no reset function on the external pin)
- Board Current Measurement state deleted and support removed
- Get Boot Time command added. Returns the time of emerging from the most recent reset
- UART Serial state machine improved with finite turnaround from receive to transmit
- Epoch transmission via UART implemented.
- RESETREAS register cleared after reading
- power management prevented in streaming state when UART connection is present
- Watchdog Timer enabled
- INIT state improved. 
    - Supplies battery information to BLE battery service. 
    - Will now allow a direct transition to the IDLE state following a reset whose reason is not SREQ and is either DOG or LOCKUP
    - Will check for low battery and enter SHUTDOWN state if empty.
- ON_DOCK to SYNTHESIZE transition no longer disables serial if UART is connected.
- SYNTHESIZE to ON_DOCK transition enables serial if no UART connection is currently present.

## 1.07

- External pin enabled (no reset function on external pin)
- 5 second settling time (constant low) now required for sticky pad application detection 
- Continual ping added to UART communications to allow asynchronous transmission by PT10 and to keep connection alive
- EEG streaming data over UART implemented

## 1.06 

- Reset function on external pin

## 1.05

- External pin enabled (no reset function on external pin)
- Storage start-up and erase functions improved for speed
    - Erase now returns immediately, and performs erase in the background
    - Epoch Discovery searches every 4KB subsector not every page
- Sticky pad application sequence improved with an extra state
- New command added to return RESETREAS register contents

## 1.03

- 15s epoch bug fixed. Active mode generates twice as many FIFOs per second, which were being used as the epoch timer
- Huru command processor unified across BLE and UART interfaces

## 1.01

- Synthesized epoch timestamps separated by 1s - currently limits synthesis to 1 epoch per second
- 1GBit flash bulk erase command fixed