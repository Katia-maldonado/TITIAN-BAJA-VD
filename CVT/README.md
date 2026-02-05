## CVT Dyno Harness & Tuning Overview

In addition to the vehicle DAQ system, we are developing a **separate electrical harness for the CVT dyno**. This setup is designed specifically for **CVT testing and tuning** in a controlled environment and does not require the same level of durability as the on vehicle harness.

The CVT dyno harness uses an **Arduino Uno with an SD card reader shield** to log data and display real time feedback during testing. The system includes the following components:

- Hall effect sensor – Primary RPM  
- Hall effect sensor – Secondary RPM  
- Hall effect sensor – Disk Brake RPM  
- Push button (test control / reset)  
- LED indicators (system status)  
- Real Time Clock (RTC)  
- Display screen for real time output  

The hall effect sensors allow us to measure **primary, secondary, and wheel RPM**, which are critical for understanding CVT behavior under different load and throttle conditions. Data is logged to an SD card for analysis, while the display provides immediate feedback during testing.

---

## Purpose of CVT Tuning

The purpose of CVT tuning is to optimize the transmission’s performance by ensuring the engine operates within its ideal RPM range throughout acceleration and load changes. By analyzing the relationship between **primary RPM, secondary RPM, and wheel RPM**, we can evaluate:

- Shift behavior and ratio changes  
- Belt slip and engagement consistency  
- Backshift response under load  
- Overall drivetrain efficiency  

This data approach allows us to make informed adjustments to **spring rates, flyweights, and CVT setup** rather than relying solely on trial and error. The CVT dyno harness provides a repeatable testing environment that directly supports on vehicle performance improvements.

---

## To Do List

- [x] Hall effect sensor – Primary RPM  
- [x] Hall effect sensor – Secondary RPM  
- [ ] Hall effect sensor – Wheel RPM  
- [x] Push button (test control / reset)  
- [x] LED indicators (system status)  
- [x] Real-Time Clock (RTC)  
- [ ] Display screen for real-time output  
