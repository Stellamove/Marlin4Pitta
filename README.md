# Marlin4Pitta 3D Printer Firmware For Pitta

![GitHub](https://img.shields.io/github/license/Stellamove/Marlin4Pitta)

This is a firmware for 3D Printer with Pitta

## Installing Marlin4Pitta
To install Marlin4Pitta on your printer you’ll first need to Download Marlin4Pitta, then use an IDE to Compile the Marlin4Pitta into a binary form and Upload it to your board. Download Marlin4Pitta source code on the [Github page](https://github.com/Stellamove/Marlin4Pitta)

![download](https://user-images.githubusercontent.com/96027590/145907300-a39be774-6594-4594-b73b-d7e76439e0f6.jpg)

## Example Configurations
Before building Marlin4Pitta you'll need to configure it for your specific hardware. Your vendor should have already provided source code with configurations for the installed firmware, but if you ever decide to upgrade you'll need updated configuration files. Marlin4Pitta will provide tested example configurations to get you started. Visit the [Configurations](https://github.com/Stellamove/Configurations) repository to find the right configuration for your hardware.

## Building Marlin4Pitta
To build Marlin4Pitta you'll need [Visual Studio Code](https://code.visualstudio.com/) and [PlatformIO](https://docs.platformio.org/en/latest//integration/ide/index.html#platformio-ide). PlatformIO extension turns Visual Studio Code into a complete IDE for compiling and developing Marlin4Pitta.

![platformio](https://user-images.githubusercontent.com/96027590/145910073-1413379d-7f93-4516-ac42-30f6231ab456.jpg)

### 1. Install Visual Studio Code
Visit the [Visual Studio Code](https://code.visualstudio.com/) page to download and install the latest Visual Studio Code for your particular platform.

### 2. Install PlatformIO extension
Head over to the [Get PlatformIO IDE](https://platformio.org/install/ide?install=vscode) page to learn how to install PlatformIO IDE in VSCode.

### 3. Open Marlin4Pitta in Visual Stdio Code with PlatformIO
You can open Marlin in Visual Studio Code by use the Open Folder command in the Visual Stdio Code's File menu.

### 4. Build, Clean
Use the bottom Status Bar icons to build or clean.

![build](https://user-images.githubusercontent.com/96027590/145912771-bc4068ba-0bb7-4cd6-96e2-744c8dde9246.jpg)

## Description of configuration changes for using Pitta
### Configuration.h
EXTRUDERS and SINGLENOZZLE have been changed because Pitta supports 8 colors using a single nozzle.
```
// PITTA
#if MMU_MODEL == PITTA_MMU
#define EXTRUDERS 8
#else
#define EXTRUDERS 1
#endif
```

```
// PITTA
#if MMU_MODEL == PITTA_MMU
#define SINGLENOZZLE
#else
//#define SINGLENOZZLE
#endif
```

Changed MMU_MODEL for pitta.
```
// PITTA
#define MMU_MODEL PITTA_MMU
```

Ender-3 V2 LCD DWIN_SET has been changed.
```
// PITTA
//#define DWIN_CREALITY_LCD           // Creality UI
#define DWIN_CREALITY_LCD_ENHANCED    // Enhanced UI
//#define DWIN_CREALITY_LCD_JYERSUI   // Jyers UI by Jacob Myers
//#define DWIN_MARLINUI_PORTRAIT      // MarlinUI (portrait orientation)
//#define DWIN_MARLINUI_LANDSCAPE     // MarlinUI (landscape orientation)
```

### Configuration_adv.h
Extruder thermal protection has been changed because filament replacement requires more time.
```
// PITTA
#define THERMAL_PROTECTION_PERIOD 60        // Seconds
#define THERMAL_PROTECTION_HYSTERESIS 10    // Degrees Celsius
```

### platformio.ini
Changed default_envs, default_src_filter for Ender-3 V2 with Pitta.
```
default_envs = STM32F103RET6_creality
```

```
default_src_filter = 
        :
-<src/feature/pitta/pitta.cpp>
```

### features.ini
Added HAS_PITTA_MMU for Pitta
```
HAS_PITTA_MMU = src_filter=+<src/feature/pitta/pitta.cpp>
```

### Conditionals_LCD.h
```
// PITTA
#define PITTA_MMU              4
        :
// PITTA
#if MMU_MODEL == PITTA_MMU
  #define HAS_PITTA_MMU 1
#endif
        :
// PITTA
#undef PITTA_MMU
        :
// PITTA
#elif HAS_PITTA_MMU
  #define E_STEPPERS      1
  #define E_MANUAL        1
#endif
```

## Upload firmware to 3D printer
3D printer require the firmware.bin file to be copied onto the onboard SD card, and then you must reboot the printer to complete the install. Firmware binary file is located in the ".pio/build/(your target board)/" folder.

![firmware](https://user-images.githubusercontent.com/96027590/145913563-e3164dec-4648-4d95-b00d-e1b66b650789.jpg)

1. Prepare SD card formated with MBR, FAT32 and 4096 allocation size.
2. Copy the firmware binary file to the root of the card.
3. Ensure that the name of the file was not previously used to update the 3D printer.
4. Turn off the 3D printer, disconnect any USB cable and insert the SD card.
5. Turn on the 3D printer, the upload firmware process will start automatically.
6. The LCD will be blank until the upload is finished (about 15 seconds).
7. If you are uploading from the original firmware or other source, please restore defaults from LCD menu.

## Upload firmware to LCD of Ender-3 V2
If you have uploaded the firmware of Ender-3 V2, you must also upload the display firmware of the LCD for Ender-3 V2. You will find the instruction to upload and upload the DWIN_SET form [LCD_Files](https://github.com/Stellamove/LCD_Files/tree/master/Ender-3%20V2).

