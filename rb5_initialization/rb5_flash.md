# Guide: Flashing Qualcomm RB5 Development Kit from Ubuntu 24.04

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
- [Summary](#summary)
  - [Supported Configurations](#supported-configurations)
  - [Limitations](#limitations)
- [Troubleshooting](#troubleshooting)
  - [Device Not Detected](#device-not-detected)
  - [Docker Container Issues](#docker-container-issues)

---

## Introduction

This guide provides step-by-step instructions for flashing the Qualcomm Robotics RB5 Development Kit from an Ubuntu 24.04 host system. Since SDK Manager cannot run directly on Ubuntu 24.04, the solution involves using Docker containers with Ubuntu 20.04 or 22.04.

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

Replace `vx.x.x` with the actual version number.

After this step, the necessary Dockerfiles (Dockerfile_20.04, Dockerfile_22.04) and SDK Manager files will be available in the current directory.

## Docker Container Setup

### Choose Target OS Version

The Dockerfile determines which Ubuntu version will be installed on the RB5:

#### For Ubuntu 20.04 on RB5 (LU2.0 Platform)

```bash
ln -sf Dockerfile_20.04 Dockerfile
sudo docker build -t ubuntu:20.04-sdkmanager .
```

#### For Ubuntu 22.04 on RB5 (Gen2 Platform)

```bash
ln -sf Dockerfile_22.04 Dockerfile
sudo docker build -t ubuntu:22.04-sdkmanager .
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

**Note:** Replace `20.04` with `22.04` if targeting Ubuntu 22.04.

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
   - **LU** for Ubuntu 20.04
   - **Gen2** for Ubuntu 22.04
5. Choose the available version
6. Start download and repack process

**Note:** This process takes approximately 30-40 minutes.

### Enter EDL Mode

Prepare the RB5 board to enter Emergency Download (EDL) mode:

1. Power off the board completely (disconnect all cables)
2. Connect the 12V power supply
3. Press and hold the F_DL button
4. Connect USB-C cable to the PC
5. Release the F_DL button

The board should now be in EDL mode.

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

## Summary

### Supported Configurations

- Ubuntu 20.04 on RB5 using LU2.0 platform (Docker Ubuntu 20.04)
- Ubuntu 22.04 on RB5 Gen2 (Docker Ubuntu 22.04)

### Limitations

- LU1.0 platform only supports Ubuntu 18.04 (not Ubuntu 20.04)
- SDK Manager requires Ubuntu 20.04 or 22.04 environment
- ModemManager must be stopped during the process

## Troubleshooting

### Device Not Detected

- Verify ModemManager is stopped: `sudo systemctl status ModemManager`
- Check USB connection
- Verify EDL mode entry (F_DL button procedure)
- Check `adb devices` output

### Docker Container Issues

- Ensure Docker daemon is running: `sudo systemctl status docker`
- Check container status: `sudo docker ps -a`
- Verify volume mounts are correct
