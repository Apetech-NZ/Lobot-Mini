# Lobot-Mini
An ESP32-based controller for FRA Antweight (SPARC Fairyweight) robots

# NOTE:
**Many aspects of this project are incomplete!** I have just created the repo as a way to gauge interest and collect feedback on the project. If there is enough interest I will progress the project by developing Arduino-based firmware for the ESP32 and an Android app to control it.

## Background
Over the last couple of years, in my spare time, I've been developing a simple low-cost robot called [Lobot](https://github.com/Apetech-NZ/Lobot). After I got involved in combat robotics I noted that the cost of the hardware, the transmitter in particular, was a barrier for some people. I realised that Lobot actually already had all the necessary functionality to control an Antweight robot, but it is designed as a standalone device with the drive motors, battery etc... mounted directly to the PCB and size optimized for hand-assembly. I stripped out all the unnecessary functionality and shrank the board as much as possible (without increasing the cost of the PCB too much and still allowing for hand-assembly). To keep the cost and size down I've kept the design relatively simple and haven't included much in the way of protection (same as most hobby-grade electronics).

## Features
- Bluetooth communication
- Wifi connectivity (just for firmware updates at this stage)
- Dual H-bridge output for driving N20 or similar brushed motors
- PWM output for a servo or ESC
