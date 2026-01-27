# Guide: Flashing Qualcomm RB5 Development Kit from Ubuntu 24.04
RB5 is already flashed and has everyhthing installed for 3d camera integration, if you ever need to flash again (not recommended) follow this guide. 
the current vzersion flashed on rb5 is ubuntu 20.04, we tried newer versions it didn't work. 
## Table of Contents

- [Introduction](#introduction)
- [Host System Preparation (Ubuntu 24.04)](#host-system-preparation-ubuntu-2404)
  - [Install ADB](#install-adb)
  - [Install Required Dependencies](#install-required-dependencies)
  - [Stop ModemManager](#stop-modemmanager)
- [SDK Manager Installation](#sdk-manager-installation)
  - [Download SDK Manager](#download-sdk-manager)
  - [Extract Package Files](#extract-package-files)
- [Docker Container Setup](#docker-container-setup)
  - [Choose Target OS Version](#choose-target-os-version)
  - [Run Docker Container](#run-docker-container)
- [Flashing Process](#flashing-process)
  - [Start SDK Manager](#start-sdk-manager)
  - [SDK Manager Configuration](#sdk-manager-configuration)
  - [Enter EDL Mode](#enter-edl-mode)
  - [Flash the Device](#flash-the-device)
- [Verification](#verification)
---

## Introduction

This guide provides step-by-step instructions for flashing the Qualcomm Robotics RB5 Development Kit from an Ubuntu 24.04 host system. Since SDK Manager cannot run directly on Ubuntu 24.04, the solution involves using Docker containers with Ubuntu 20.04.
## Preparation : 
Create an account on thudercomm 
[https://www.thundercomm.com/register/](https://www.thundercomm.com/register/)

## Host System Preparation (Ubuntu 24.04)
### Install ADB

First, install Android Debug Bridge (ADB) to communicate with the device:

```bash
sudo apt install adb
```

Verify the installation and device detection:

```bash
adb devices
```

Ensure the device is detected before proceeding.

### Install Required Dependencies

Install all necessary packages for SDK Manager and Docker:

```bash
sudo apt update
sudo apt install -y coreutils fakechroot fakeroot kmod \
    libc6-arm64-cross qemu-user-static wget udev \
    openssh-server docker.io unzip git
```

### Stop ModemManager

ModemManager must be stopped to allow proper recognition of the RB5 in EDL mode:

```bash
sudo systemctl stop ModemManager
```

## SDK Manager Installation

### Download SDK Manager

Download the SDK Manager from Thundercomm:

[https://www.thundercomm.com/product/qualcomm-robotics-rb5-development-kit/#sdk-manager](https://www.thundercomm.com/product/qualcomm-robotics-rb5-development-kit/#sdk-manager)

Look for the **Qualcomm Robotics RB5 Development Kit - Thundercomm** section.

### Extract Package Files

Extract and install the SDK Manager package. Note that this step extracts the SDK Manager files and Dockerfiles to your system, but does not run SDK Manager on Ubuntu 24.04 (which is incompatible). SDK Manager will run inside the Docker container in the next steps.

```bash
unzip TC-sdkmanager-4.2.0.zip
cd TC-sdkmanager-4.2.0
sudo dpkg -i tc-sdkmanager-vx.x.x_amd64.deb
```

After this step, the necessary Dockerfiles (Dockerfile_20.04, Dockerfile_22.04) and SDK Manager files will be available in the current directory.

## Docker Container Setup

### Choose Target OS Version

The Dockerfile determines which Ubuntu version will be installed on the RB5:

#### For Ubuntu 20.04 on RB5 (LU2.0 Platform)

```bash
ln -sf Dockerfile_20.04 Dockerfile
sudo docker build -t ubuntu:20.04-sdkmanager .
```

### Run Docker Container

Launch the container with appropriate privileges and volume mounts:

```bash
sudo docker run -v /home/${USER}:/home/hostPC/ \
    --privileged \
    -v /dev/:/dev \
    -v /run/udev:/run/udev \
    -d \
    --name sdkmanager_container \
    -p 36000:22 \
    ubuntu:20.04-sdkmanager
```

## Flashing Process

### Start SDK Manager

Execute SDK Manager inside the Docker container:

```bash
sudo docker exec -it sdkmanager_container sdkmanager
```

### SDK Manager Configuration

Follow these steps in the SDK Manager interface:

1. Log in with your Thundercomm account credentials
2. Select working directory: `/home/hostPC/rb5_build`
3. Select product: **RB5**
4. Select platform:
   - **LU2.0** for Ubuntu 20.04
5. Choose the available version (latest)
6. Start download process

**Note:** This process takes approximately 30-40 minutes.

### Enter EDL Mode

Prepare the RB5 board to enter Emergency Download (EDL) mode:

1. Power off the board completely (disconnect all cables)
2. Connect the 12V power supply
3. Press and hold the F_DL button
4. Connect USB-C cable to the PC
5. Release the F_DL button

The board should now be in EDL mode. 
check if it is in EDL mode 
``` bash 
adb devices
```
You should now see the device in EDL mode 

### Flash the Device

In SDK Manager:

1. Type `2` to select flash option
2. Select **Flash full build**
3. The flashing process will start automatically
4. Wait for completion (the board will reboot automatically)

## Verification

After the board reboots, verify successful installation:

```bash
adb wait-for-device shell
```

If a shell prompt appears, the flash was successful.
