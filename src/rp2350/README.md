# Raspberry Pi Pico2's firmware

## How to build

1. Install PlatformIO
    - Follow the instructions at https://platformio.org/install/ide?install=vscode

2. Install RP2350 board support
    - Open the PlatformIO extension in Visual Studio Code.
    - Go to the "Platforms" tab.
    - Click on "Advanced Installation" and enter the following URL:
      ```
      https://github.com/maxgerhardt/platform-raspberrypi.git
      ```
    - Click "Install" to install the RP2350 board support.

3. Open this directory in Visual Studio Code

4. Build the firmware
    - Press `Ctrl + Alt + B` to build the firmware.

5. Upload the firmware
    - Connect your Raspberry Pi Pico2 to your computer.
    - Press `Ctrl + Alt + U` to upload the firmware.
