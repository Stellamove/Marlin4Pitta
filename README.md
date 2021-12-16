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

## Upload firmware to 3D printer
3D printer require the firmware.bin file to be copied onto the onboard SD card, and then you must reboot the printer to complete the install. Firmware binary file is located in the ".pio/build/(your target board)/" folder.

![firmware](https://user-images.githubusercontent.com/96027590/145913563-e3164dec-4648-4d95-b00d-e1b66b650789.jpg)
