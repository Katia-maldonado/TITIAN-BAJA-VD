# Underclassmen Code Submissions

This folder is used for underclassmen to upload **code contributions** related to the **CVT dyno** and **on vehicle DAQ system**. To keep the repository organized and easy to review, please follow the naming and documentation rules below.

---

## File Naming Convention

All uploaded code files must follow this format:


### Examples
- `v9_CVT_KatiaM`
- `v9_DAQ_KatiaM`

### Rules
- Use the current version number (ex: `v9`)
- Specify the system:
  - `CVT` for CVT dyno code
  - `DAQ` for on-vehicle DAQ code
- Use your **first name + last initial**
- Do not use spaces in file names

---

## Required Code Header Comment

Each code file **must include a comment block at the very top** describing what was changed and what sensors are used. This helps with review and future debugging.

### Required Format (Copy & Paste)

```
/*
Author: Your Name
Version: version number (ex:v9)
System: CVT or DAQ

Description:
- Briefly describe what you added or modified in this version

Sensors Included:
- List all sensors used in this code
*/
/*
Author: Katia Maldonado
Version: v9
System: CVT

Description:
- Added live screen output for RPM values
- Improved hall effect sensor debounce

Sensors Included:
- Hall effect sensor (Primary RPM)
- Hall effect sensor (Secondary RPM)
- Hall effect sensor (Wheel RPM)
*/
