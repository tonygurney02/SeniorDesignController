# SeniorDesignController

This repository contains early-stage C code written for an STM32F3 Discovery board as part of a senior design project. The code was developed using STM32CubeIDE to control a motor-based system for automated dunnage dropping. Although the motor-based approach was later scrapped, this version showcases my work on GPIO control, debouncing, and user interaction using onboard peripherals.

## 🛠️ Project Overview

- Developed on: **STM32F3Discovery** board  
- IDE: **STM32CubeIDE**  
- Language: **C**

## 🔧 Functionality

- The **user button** (on the board) is used to cycle through light states, each representing a different amount of dunnage to be dropped.
- A secondary button connected to **PD7** triggers the simulated dunnage drop.
- The system includes:
  - **Debouncing** logic for the PD7 input
  - **LED indicators** to show the selected dunnage amount and dropping confirmation

## 📁 File Structure

- Main logic is located in:  
  `Core/Src/senior_design_controller.c`

> Note: This version represents an early prototype and is no longer the basis for the final project implementation.

---

This project is part of my senior design portfolio, and it reflects my embedded systems development experience using STM32 microcontrollers. Feedback is welcome!


