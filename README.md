# CC2541_BLE_NetCod: Network Coding in BLE Networks

This repository contains the source code and experimental data for the paper **"Network Coding in BLE Networks"**, which investigates the application of network coding–based diversity techniques to enhance the reliability of Bluetooth Low Energy (BLE) communications.

It provides a complete implementation of cooperative communication schemes, including **Decode-and-Forward (DAF)**, **Binary Network Coding (BNC)**, and **Dynamic Network Coding (DNC)**, on resource-constrained, low-cost BLE hardware (Texas Instruments CC2541).

---

## Overview

Wireless communication in the 2.4 GHz ISM band is subject to significant interference and multipath fading.  
This project demonstrates that by enabling cooperation between nodes using **Network Coding (NC)**, the **Packet Delivery Ratio (PDR)** can be significantly improved compared to traditional non-cooperative approaches, even when using simple hardware with non-optimized antennas.

---

## Key Features

### Cooperative Schemes
- **NoC** – No Cooperation (baseline)  
- **DAF** – Decode-and-Forward (relaying)  
- **BNC** – Binary Network Coding (XOR-based)  
- **DNC** – Dynamic Network Coding using Galois Field arithmetic (GF(2^q))

### Centralized Control
A robust synchronization protocol where the **Destination** node orchestrates the experiment, dynamically switching:
- cooperation schemes  
- power levels  
- repetition counts  

### Real-Time Encoding
Finite field arithmetic and matrix operations implemented directly on the **TI CC2541** microcontroller.

### Performance Metrics
- Logs **RSSI**
- Logs **PDR**
- Allows analysis of link quality and reliability

---

## Hardware Requirements

The code is designed for the **Texas Instruments CC2541 System-on-Chip (SoC)**.  
Experiments were conducted using **JDY-08 Bluetooth modules**, low-cost breakout boards for the CC2541.

- **Microcontroller:** TI CC2541
- **IDE:** IAR Embedded Workbench for 8051  

---

### Main File Descriptions
- **BLE_Broadcaster_cc254x.c**  
  Handles advertisement broadcasting, scanning for partner packets, and performing network coding operations.

---

## Usage

### 1. Flash the Firmware
- Program **one** node as the **Destination (Controller)**  
- Program **two or three** nodes as **Sources/Relays (Broadcasters)**  

### 2. Operation
1. Power on all devices.  
2. The Destination broadcasts a configuration packet to start the test cycle.  
3. Source nodes synchronize and execute the broadcast and cooperation phases according to the received parameters  
   (e.g., Scheme = DNC, Power = 0 dBm).  
4. The Destination logs received packets and RSSI via UART/Serial.

---

## Citation

If you use this code or data in your research, please cite:

```bibtex
@article{Monteiro2025_Access,
  author  = {Marcos Eduardo Pivaro Monteiro and Pedro Carvalho da Fonseca Guimarães 
             and Jamil Farhat and Daniel Fernando Pigatto and Eduardo Nunes dos Santos},
  title   = {Network Coding in {BLE} Networks},
  journal = {IEEE Access},
  year    = {2025},
  doi     = {10.1109/ACCESS.2025.XXXXXXX}
}
