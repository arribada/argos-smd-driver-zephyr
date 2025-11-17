# Argos SMD Driver for Zephyr

This is a Zephyr driver for the Argos SMD module based on STM32WL from Arribada. [Hardware repository](https://github.com/arribada/argos-smd-hw).

## Architecture

The Argos SMD module is a Serial Peripheral and is connected to the Zephyr host via UART. The driver uses the UART Polling API for sending data to the argos smd and the Interrupt API for receiving data.


### Commands

Commands are sent to the ARGOS SMD module following the Kineis AT command:
    - Commands start by AT+
    - Answer start by '+' and finish by end of line
    - Get command is terminated by "=?"   
    - Set command is terminated by "=VALUE"
```bash
# commands available:

# General commands
AT+VERSION=?          # AT_VERSION
AT+PING=?             # AT_PING
AT+FW=?               # AT_FW
AT+ADDR=?             # AT_ADDR
AT+ID=?               # AT_ID
AT+SECKEY=?           # AT_SECKEY
AT+SN=?               # AT_SN
AT+RCONF=?            # AT_RCONF
AT+SAVE_RCONF=?       # AT_SAVE_RCONF
AT+LPM=?              # AT_LPM
AT+MC=?               # AT_MC
AT+TCXO_WU=?          # AT_TCXO_WU

# User data commands
AT+TX='MSG'           # AT_TX

# Certification commands
AT+CW=?               # AT_CW 

# Date/time commands
AT+UDATE=?            # AT_UDATE

# MAC commands
AT+KMAC=?             # AT_KMAC

# Prepass
AT+PREPASS_EN=?       # AT_PREPASS_EN
```

# Requirements #
This repo is for Windows Machines using Windows 11 and above.
It is assumed that VSCode is already installed and that you have SSH access to Github. 

## WSL & Docker Setup ##
The development environment is defined in the Dockerfile present in the .devcontainer folder. It builds an image that can be run as a Dev Container on WSL2. WSL2 enables a Linux environment to run on a Windows Machine. 

1. Follow this guide: https://learn.microsoft.com/en-us/windows/wsl/setup/environment to install WSL2. Ensuring that you:
  1. Install WSL2 with Ubuntu
  2. Create a Linux User
  3. Update and upgrade the packages
3. Install Docker Desktop using the following guide: https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers#install-docker-desktop
  1. Please follow up to the end of the Install Docker Desktop section
  2. It can be useful to enable "Start Docker Desktop when you sign in to you computer" in the Docker Desktop settings. 
4. Download and install the latest version of [wsl-usb-gui](https://gitlab.com/alelec/wsl-usb-gui/-/releases) This allows USB pass-through from Windows to WSL.
  1. Once installed, open it and allow it to install its dependencies. 

## VS Code Dev Containers ##
1. Install the Dev Containers Extension: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers 

# Getting Started #
This repo hasn't been tested on Windows machines, initially please follow the Mac Tested steps below. If these don't work please follow the alternative steps.

## Tested Steps ##
1. Open WSL
2. Setup SSH
3. Run `git clone git@github.com:arribada/linkitv4.git` in WSL 
4. Open VSCode and click the Blue Box in the bottom left hand corner and click `WSL`
5. Open the Repo with `File -> Open Folder`
6. On opening you should be presented with an open within a dev container prompt. If not press F1 to open the command palate and type: `Open Folder in Dev Container` to find the command.
7. Docker will build the image, this can take some time, follow the Dev Container prompt to show logs.
8. Go inside example/simple build for your breakout board ex: `west build -b adafruit_feather_nrf52840`
9. Flash with: `west flash`
10. Read trace with RTT debugger: `west rtt`