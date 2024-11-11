
# Automated Cutting Machine with PID Control System

### Project Overview

The **Automated Machine** project is designed to control the precision cutting of reflective and elastic bands, featuring a **PID control system** for high accuracy and adjustable sizing and quantity. This machine improved processing speed by up to 450% compared to its predecessor, the **Patterning Stitching Machine**. Furthermore, the error rate was reduced to less than 1%, achieving 0.9901%.

### Key Features
- **Adjustable Size and Quantity:** The system allows users to set desired sizes and quantities for elastic band cuts.
- **PID Control:** Maintains accuracy with PID parameters (`kp`, `ki`, `kd`) to ensure precise positioning.
- **IR Remote Control:** Simplifies user input and navigation through the system's menu using an IR remote.
- **Encoder Feedback:** Enables the system to track position and adjust accordingly.

### Usage

The main loop of the code reads IR signals and controls the machine according to the selected menu options. There are five main menus in the system:

1. **Menu 1:** Displays the current step count.
2. **Menu 2:** Allows the user to set the desired band length.
3. **Menu 3:** Sets the quantity of bands to cut.
4. **Menu 4:** Confirms the selections.
5. **Menu 5:** Initiates the cutting process.

### Code Structure

The code consists of the following main components:
- **PID Calculation**: Uses proportional, integral, and derivative terms to determine motor control signals.
- **Motor Control**: Adjusts motor speed and direction based on PID output.
- **Encoder Feedback**: Tracks the position of the cutting mechanism.
- **Menu Control**: Allows users to navigate the menu using IR remote input.

### PID Formula Derivation

```cpp
long currT = micros();
float deltaT = ((float)(currT - prevT)) / (1.0e6);
prevT = currT;

// Error
long e = pos - act;
float dedt = (e - eprev) / (deltaT);
eintegral += e * deltaT;

// Control Signal
float u = kp * e + kd * dedt + ki * eintegral;

// Motor Power
pwr = fabs(u);
pwr = (pwr > 255) ? 255 : (pwr < 65 && pwr > 3) ? 70 : pwr;

// Direction
int dir = (u < 0) ? -1 : 1;

// Store previous error
eprev = e;
```

### Wiring & Pin Assignments

| Component    | Pin             |
|--------------|-----------------|
| IR Receiver  | 10              |
| Relay        | 6               |
| Sensor       | 5               |
| Button       | A0              |
| PWM Output   | 7               |
| Motor Input A| 8               |
| Motor Input B| 9               |

### LinkedIn Contact
If you have questions or feedback, please feel free to reach out through my [LinkedIn](https://linkedin.com/in/rizkisaputrasembiring).

![Automated Cutting Machine](https://github.com/user-attachments/assets/be28f9dc-3231-4eba-86d7-0951340429a2)
