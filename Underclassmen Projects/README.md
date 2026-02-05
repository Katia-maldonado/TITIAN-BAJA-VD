## Underclassmen Projects (Coding)

I have a few underclassmen projects for anyone interested in writing code for **CVT tuning** and the **main DAQ harness** that will be installed on the car. These projects are a great opportunity to gain hands on experience with Arduino based data acquisition and sensor integration.

For coding related work, please reference **version 8 (v8)** of the code located in the **DAQ** or **version 9 (v9)** in the **CVT** folders. This version should be used as the baseline for development.

If you’re interested, **reach out to Katia on Discord**, and she can answer any questions you may have and help you get started.

---

### CVT DYNO

- [ ] Add code to output **live data** onto a screen  
- [ ] Display outputs for:
  - [ ] Hall effect sensor (Primary RPM)  
  - [ ] Hall effect sensor (Secondary RPM)  
  - [ ] Hall effect sensor (Disk Brake RPM)  

---

### DAQ (On Vehicle)

- [ ] Code the **IMU (MPU-6050)** to record and send its data to the Arduino  
- [ ] Log IMU data onto an SD card using the **SD Card Reader Shield** (so it can be exported to Excel / MATLAB later)  
- [ ] IMU signals to record:
  - [ ] Angular velocity (gyro): **X, Y, Z**
  - [ ] Linear acceleration (accelerometer): **X, Y, Z**

> Note: The MPU-6050 is a 3-axis accelerometer and gyroscope. There are many examples available on GitHub and YouTube that can be used as references.

---

### Driver Dash Display

We are also exploring the addition of a **driver dash display** to provide live feedback during vehicle operation. The goal is to give the driver real time readings without relying on post run data analysis.

Planned display outputs include:
- Vehicle speed  
- Secondary RPM  
- CVT belt temperature (IR sensor)  

Due to the off road environment, the display must be **IP67-rated** to withstand dust, water, mud, and vibration. Many off the shelf IP67 displays are expensive, so part of this project is to find **cost effective display solutions** that can meet durability requirements.

This project may include:
- Researching low cost display options compatible with Arduino  
- Designing or selecting a **waterproof enclosure** for the screen  
- Integrating the display with the existing DAQ system  
- Ensuring readability in outdoor and high vibration conditions  

---

## Underclassmen Projects (Hands On)

For those interested in hands on work, there are opportunities to help test and validate the sensors we are using by connecting them to an Arduino and breadboard while we wait for final parts to be delivered. This is a great way to get familiar with the hardware and basic wiring before full integration.

A future project will involve assisting with **harness assembly and integration** once all components arrive, including helping connect and route the wiring for the CVT dyno and on vehicle DAQ systems. Interested students should reach out to Katia for more information.
