## Code Notes & Versioning

**Note:** **Version 8 (v8)** is the most current and stable version of the DAQ code. It has been tested using an **Arduino and breadboard** and should be used as the baseline for any new development.

All new code files must follow the updated naming convention:

Example:
- `v9_DAQ_KM`

---

## Required Code Header Comment

Each DAQ code file must include a **comment block at the very top** of the file describing what the code does and which sensors are used. This helps keep the repository organized and makes code review easier.

### Required Format (Copy & Paste)

```cpp
/*
Version: v#
Author: Name
System: DAQ

Description:
- Brief description of what was added or changed

Sensors Included:
- List all sensors used in this code
*/
