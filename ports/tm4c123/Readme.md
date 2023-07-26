# MicroPython Port for Tiva Launchpad

<img src="https://raw.githubusercontent.com/rk-exxec/micropython/tiva_from_stable/logo/trans-logo.png" height="200" /><img src="https://www.ti.com/diagrams/med_ek-tm4c123gxl_tivalp_angle_new.jpg" height="200" />


#### Table of contents
1. [Build](#build)
   - [Ubuntu](#ubuntu)
   - [Windows](#windows)
2. [Flashing](#flashing)
3. [Debug](#debug)
   - [TI CodeComposer Studio](#ccs)
   - [VSCode](#vscode)
4. [REPL](#connecting-to-repl)
5. [Usage](#usage)
   - [SD-Card](#sd-card)
   - [Pin](#pin)
   - [UART](#uart)
   - [SPI](#spi)
   - [Timer](#timer)
   - [PWM](#pwm)


## Build
### Ubuntu
   Works for Ubuntu / Debian
1. Clone git repo:
   ```bash
   cd <your target directory>
   git clone https://github.com/rk-exxec/micropython.git 
   cd ./micropython
   git checkout tiva_from_stable
   ```
2. Install Toolchain:
   ```bash
   source tools/ci.sh && ci_tm4c123_setup
   ```
3. Build:
   ```bash
   cd ./ports/tm4c123
   make
   ```
   
### Windows

1. Install [MinGW]( https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download)  
      **Make sure it is added to your PATH**  
   **OR** install [WSL](https://docs.microsoft.com/de-de/windows/wsl/install-win10#manual-installation-steps)
2. Install [ARM toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) for Windows to debug and MinGW to compile  
      **Make sure it is added to your PATH**
3. Clone git repo and checkout branch `tiva_from_stable`
4. Download and install [TivaWare](https://www.ti.com/tool/SW-TM4C) or extract contents to `micropython/ports/tm4c123/tivaware`  
   **OR** if you are using WSL, do Step 2 of [Ubuntu Instructions](#ubuntu)  
5. Build using WSL or MinGW terminal:
   ```bash
   cd ./ports/tm4c123
   make
   ```
   Use param `TIVAWARE_LIB="C:\ti\TivaWare_2.2.0.XXX" ` with make if you installed TivaWare
   
## Flashing

1. Download & install UniFlash: https://www.ti.com/tool/UNIFLASH 
   - Create new Configuration
   - Device: EK-TM4C123GXL
   - Connection: Stellaris In-Circuit Debug Interface
   - Start
   - Load Flash image: `build/firmware.bin`
   - Reset Actions: Core Reset, Check `Execute selected reset after program load`
   - Load Image

## Debug

### CCS

   * Download CodeComposerStudio and import the source code as external project
   * Configure the debugger to load the uPY_TM4C123.axf

### VSCode

1. Download and install Visual Studio Code with these extensions:
   - ARM Code from Dan C Underwood
   - Cortex-Debug from marus25
   - Python 
   - C/C++
   - Native Debug from WebFreak
   - LinkerScript from ZixuanWang

2. Download & install drivers: https://www.ti.com/lit/zip/slac632  
   (Windows only, also included in the TivaWare)
   - connect board
   - right click Start button and click Device-Manager
   - there should be 2 unknown devices
   - Click update drivers and use manual select
   - navigate to the top level folder containing the drivers
   - Windows selects the driver on its own
   - repeat for other device
   
3. Install OpenOCD
   - Windows/WSL: https://gnutoolchains.com/arm-eabi/openocd/  
      **Add it to your PATH**
   - Ubuntu/Debian: `sudo apt install openocd`
   
4. In VSCode debugging menu, click new config and replace contents of your `launch.json` with this:
   ```json
   {
      "version": "0.2.0",
      "configurations": [
         {
            "name": "Cortex Debug",
            "cwd": "${workspaceRoot}",
            "executable": "./build/firmware.axf",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "openocd",
            "runToEntryPoint": "tm4c_main",
            "configFiles": [
                "board/ek-tm4c123gxl.cfg"
            ]
         }
    ]
   }
   ```
      **Make sure to set the `configFiles` path correctly.**
      
 5. Start the debugging (after building the project)

## Connecting to REPL

You can connect to the REPL via UART over USB. You need to install the drivers for this to work!

Use these settings:
* Baudrate = 115200
* Data bits = 8
* Stop bits = 1
* Parity = none
* Flowcontrol = none

I recommend using **minicom** on Linux or **PuTTY** on Windows.

Alternatively you can use [**mpfshell**](https://github.com/wendlers/mpfshell):
```bash
python -m pip install mpfshell
```

## Usage

Currently, these Modules are available:
* umachine
  * Pin
  * SPI
  * UART
  * SDCard
  * IRQ
  * DMA
* uos
---
### SD-Card

Connect the SD-Card in SPI-mode according to this table:  

| Board Pin | SD-Pin (SPI / SDIO)    |
| --------: | :--------------------- |
|       PE4 | Card Detect (CD / DET) |
|       PA2 | Clock (SCK / CLK)      |
|       PA3 | Chip Select (CS / D3)  |
|       PA4 | Data Out (MISO / D0)   |
|       PA5 | Data In (MOSI / CMD)   |

If your SD Card adaptor does not have a CD Pin, you need to connect it to Ground.

---
### Pin

See also: 
<http://docs.micropython.org/en/latest/library/machine.Pin.html>

    machine.Pin(id, mode, pull, *, value, drive, alt)

| Parameter | Function                                         | value range                                  | default |
| --------: | :----------------------------------------------- | :-------------------------------------------- | :----------- |
|        id | Pin ID as Number, String or Tuple \[port, pin\] | P\[A..F\]\[0..8\]                             | \-           |
|      mode | Pin mode                                        | IN, OUT, OPEN\_DRAIN, ALT                     | \-           |
|      pull | PullUp or  Down                                 | PULL\_NONE, PULL\_UP, PULL\_DOWN, OPEN\_DRAIN | PULL\_NONE   |
|     value | Pin value                                       | 0, 1                                          | 0            |
|     drive | Strength of the pad driver                          | LOW\_POWER, MED\_POWER, HIGH\_POWER           | LOW\_POWER   |
|       alt | ID of the alternative function                    | 0..15                                         | \-           |

#### Special function

    <Pin>.unlock() 

This function unlocks special pins on the Launchpad. This needs to be done to reassign another AF to these pins.
\<Pin\> is the object of the pin you wish to unlock.  
If you try to change a locked pin, you will see an error message.
---
### UART

See also:
<http://docs.micropython.org/en/latest/library/machine.Pin.html>



    machine.UART(id, baudrate, bits, parity, stop, *, ...)

|      Parameter | Function                            | value range | default |
| -------------: | :---------------------------------- | :----------- | :----------- |
|             id | UART ID als Zahl                    | 0..7         | \-           |
|       baudrate | Baudrate                            | bis 5Mbits   | \-           |
|           bits | word length                           | 5..8         | 8            |
|         parity | parity type                        | ODD, EVEN    | NONE         |
|           stop | stop bits                           | 1, 2         | 1            |
|           flow | flowcontrol (currently broken)                        | ~~RTS, CTS~~     | NONE         |
|        timeout | timeout in ms  | >= 0    | 10           |
|  timeout\_char | timeout per char in ms | >= 0    | 0            |
| read\_buf\_len | read buffer size               | 0    | 64           |

UART0 is used by the command line. Changing this unit may break the command line  
UART1 is currently not available
---
### SPI

See also:
<http://docs.micropython.org/en/latest/library/machine.SPI.html>

    SPI(id, mode, *, baudrate, polarity, phase, protocol, bits, firstbit, fss, dma, sck, mosi, miso)

|       Parameter | Function                        | value range               | default |
| --------------: | :------------------------------ | :------------------------- | :----------- |
|              id | Unit number           | 0..3                       | 0            |
|            mode | SPI Mode                       | MASTER, SLAVE, SLAVE\_OD   | MASTER       |
|        baudrate | Clock rate    | \<25M Master, \<6e6 Slave | 5e5          |
|           phase | SPI phase       | FIRST\_EDGE, SECOND\_EDGE  | FIRST\_EDGE  |
|        polarity | SPI polarity      | IDLE\_LOW, IDLE\_HIGH      | IDLE\_LOW    |
|        protocol | SSI protocol                       | SPI0..SPI3, MICROWIRE, TI  | SPI0         |
|            bits | word length                       | 4..16                      | 8            |
|        firstbit | bit order | MSB, LSB                   | MSB          |
|             fss | chip select/ frame control      | FSS\_SOFT, FSS\_HARD       | FSS\_HARD    |
|             ~~dma~~ | use DMA? (not implemented)              | ~~True, False~~                |     False         |
| ~~sck, mosi, miso~~ | (not implemented)       |                            |              |
 
If *protocol* is given, *phase* and *polarity* will be **ignored**!

---
### Timer

See also: 
<https://docs.micropython.org/en/latest/library/machine.Timer.html>

    Timer(TimerBlockID,Timer,*,mode=PERODIC,freq=None,ticks=0,prescaler=0)

| Parameter | Function                                         | value range                                     | default       |
| --------: | :----------------------------------------------- | :--------------------------------------------   | :-----------  |
| TimerBlockID | Timerblock id as number                      | \[0..5\] normal, \[6..11] wide                   | \-           |
| Timer     | Timer in timerblock                             | Timer.A,Timer.B, Timer.A\|Timer.B (concatenated) | \-           |
|      mode | Timer mode                                       | Timer.PERIODIC, Timer.ONE_SHOT                   | Timer.PERIODIC|
|      freq | Timer frequency                                  | [int],[float]*                                   | None   |
| ticks     | Exact value in RELOAD reg                       | [int]*                                          | 0 |
| prescaler | Exact value in PRESCALER reg                    | [int]*                                           | 0 |

*actual values depend on the timer used, but there are precise error texts, telling the user when the input is out of range.

A callback function for the timeout interrupt is added by calling the .irq() method:

    Timer.irq(*,trigger=Timer.IRQ_TIMEOUT, handler=None)
There are also get/set methods:
```
   Timer.frequency(*,freq)
   Timer.ticks(*,ticks)
   Timer.prescaler(*,prescaler)
```


---
### PWM
### Initialization example
#### With Pin object:

    from umachine import Pin, PWM
    pin = Pin('PD0', Pin.OUT)
    pwm = PWM(pin, freq = 1000, duty = 50)

#### With Pin identifier:

    from umachine import PWM
    pwm = PWM('PD0', freq = 1000, duty = 50)

#### With PWM module identifier (see https://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf#page=1347&zoom=100,76,119):

    from umachine import PWM
    pwm = PWM(PWM.M1PWM0, pin = 'PD0', freq = 1000, duty = 50)

<details>
   <summary>PWM Module content</summary>

   ### Constructors
   ---
      umachine.PWM(dest, [pin, active, invert, freq, duty, db_falling, db_rising, mode, irq, irq_mode])

   Construct and returns a new PWM object with the following parameters:
   - dest is a Pin object, the string identifier of a pin or one of the following:
      - M0PWM0
      - M0PWM1
      - M0PWM2
      - M0PWM3
      - M0PWM4
      - M0PWM5
      - M0PWM6
      - M0PWM7
      - M1PWM0
      - M1PWM1
      - M1PWM2
      - M1PWM3
      - M1PWM4
      - M1PWM5
      - M1PWM6
      - M1PWM7

   For more parameters, see PWM.init()

   Returns the PWM object


   ### Methods
   ---
      PWM.init([pin, active, invert, freq, duty, db_falling, db_rising, mode, irq, irq_mode])

   Initializes the PWM object with the following parameters:
   - pin is the output pin object, or the string identifier of the pin if a PWM generator id is used for initialization. May be None or omitted to use pwm without outputting through a pin
   - active specifies whether the state of the output pin is active
   - invert inverts the state of the output pin
   - freq specifies the frequency of the PWM signal
   - duty specifies the duty cycle percentage of the PWM signal
   - db_falling specifies the deadband delay in clock ticks on a falling edge
   - db_rising specifies the deadband delay in clock ticks on a rising edge
   - mode specifies the mode of the PWM object. It is the logical OR of the following:
      - PWM_GEN_MODE_DOWN or PWM_GEN_MODE_UP_DOWN to specify the counting mode
      - PWM_GEN_MODE_SYNC or PWM_GEN_MODE_NO_SYNC to specify the counter load and comparator update synchronization mode
      - PWM_GEN_MODE_DBG_RUN or PWM_GEN_MODE_DBG_STOP to specify the debug behavior
      - PWM_GEN_MODE_GEN_NO_SYNC, PWM_GEN_MODE_GEN_SYNC_LOCAL, or PWM_GEN_MODE_GEN_SYNC_GLOBAL to specify the update synchronization mode for generator counting mode changes
      - PWM_GEN_MODE_DB_NO_SYNC, PWM_GEN_MODE_DB_SYNC_LOCAL, or PWM_GEN_MODE_DB_SYNC_GLOBAL to specify the deadband parameter synchronization mode
   - irq is the interrupt handler to attach. This function receives the status of the interrupt register via its first argument of type int
   - irq_mode specifies interrupts and triggers to act upon. The irq_mode is the logical OR of the following: 
      - PWM_INT_CNT_ZERO
      - PWM_INT_CNT_LOAD
      - PWM_INT_CNT_AU
      - PWM_INT_CNT_AD
      - PWM_INT_CNT_BU
      - PWM_INT_CNT_BD
      - PWM_TR_CNT_ZERO
      - PWM_TR_CNT_LOAD
      - PWM_TR_CNT_AU
      - PWM_TR_CNT_AD
      - PWM_TR_CNT_BU
      - PWM_TR_CNT_BD

   Returns the PWM object


   ---
      PWM.deinit()

   Deinitializes the PWM object

   Returns None


   ---
      PWM.active([value])

   Sets the output of a PWM object
   - value is the desired value

   Returns whether the output is active


   ---
      PWM.invert([value])

   Sets the output inversion of a PWM object
   - value is the desired value

   Returns whether the output is inverted


   ---
      PWM.clock_divider[value])

   Sets the global clock divider for the PWM block.
   - value is the desired value, one of the following:  
      - CLK_DIV_1
      - CLK_DIV_2
      - CLK_DIV_4
      - CLK_DIV_8
      - CLK_DIV_16
      - CLK_DIV_32
      - CLK_DIV_64

   Returns a tuple of the current clock divider and the PWM frequency of the module


   ---
      PWM.freq([value])

   Sets the frequency of a PWM object
   - value is the desired value

   Returns the current frequency


   ---
      PWM.duty([value])

   Sets the duty cycle percentage of a PWM object
   - value is the desired value

   Returns the current duty cycle


   ---
      PWM.sync([pwm, *])

   Syncs the provided PWM objects
   - pwm are pwm objects to sync

   Returns None


   ---
      PWM.db([db_falling, db_rising])

   Sets the deadband delay of a PWM object. This delay is being shared within a generator. The deadband values are clock ticks. The PWM clock divider applies
   - db_falling is the desired deadband delay for a falling edge
   - db_rising is the desired deadband delay for a rising edge

   Returns a tuple of the current deadband delay values


   ---
      PWM.irq([callback, mode])

   Sets the interrupt callback and mode for the PWM object
   - callback is the interrupt callback function to attach. The callback function receives the value of the interrupt status register via its first argument of type int. The triggered interrupt will be cleared automatically after exiting the callback.
   - mode specifies interrupts and triggers to act upon. It is the logical OR of the following: 
      - PWM_INT_CNT_ZERO
      - PWM_INT_CNT_LOAD
      - PWM_INT_CNT_AU
      - PWM_INT_CNT_AD
      - PWM_INT_CNT_BU
      - PWM_INT_CNT_BD
      - PWM_TR_CNT_ZERO
      - PWM_TR_CNT_LOAD
      - PWM_TR_CNT_AU
      - PWM_TR_CNT_AD
      - PWM_TR_CNT_BU
      - PWM_TR_CNT_BD

   Returns a tuple of the current callback and mode.

</details>  
  

