
---

# LED Brightness Control Using Potentiometer

## Overview

This project demonstrates how to control the brightness of an LED using a potentiometer and Pulse Width Modulation (PWM) on a microcontroller. By reading the analog voltage from the potentiometer, the microcontroller adjusts the PWM duty cycle to vary the LED's brightness. This implementation is done in C and is compatible with Infineon's PSoC™ microcontrollers using the ModusToolbox™ development environment.

## Features

- **Analog Input Reading**: Reads the analog voltage from a potentiometer using the microcontroller's ADC.
- **PWM-Based Brightness Control**: Adjusts LED brightness by varying the PWM duty cycle based on the potentiometer's position.
- **Modular Code Structure**: Clean and organized code for easy understanding and modification.
- **Educational Purpose**: Serves as a foundational example for beginners to understand ADC and PWM in embedded systems.

## Project Structure

```
LED-Brighness-using-potentiometer/
├── main.c       // Main application code
└── README.md    // Project documentation
```

## Requirements

- **Hardware**:
  - Infineon PSoC™ microcontroller development board (e.g., CY8CKIT-062-BLE)
  - LED connected to a PWM-capable GPIO pin
  - 10kΩ Potentiometer connected to an analog input pin
  - USB cable for programming and power

- **Software**:
  - [ModusToolbox™](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/) installed on your development machine
  - Serial terminal application (e.g., PuTTY, Tera Term) for UART communication (optional)

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/ViralPatel-19/LED-Brighness-using-potentiometer.git
```

### 2. Open the Project in ModusToolbox™

- Launch **ModusToolbox™**.
- Click on **"Import Application"**.
- Navigate to the cloned repository and select the project.

### 3. Configure the ADC and PWM Components

- Open the Device Configurator in ModusToolbox™.
- **ADC Configuration**:
  - Add and configure an ADC channel to read the potentiometer's voltage.
  - Assign the analog input pin connected to the potentiometer.
- **PWM Configuration**:
  - Add and configure a PWM component:
    - **PWM Frequency**: Set as desired (e.g., 1 kHz)
    - **Duty Cycle**: Initialize to a default value (e.g., 50%)
    - **Output Pin**: Assign to the GPIO pin connected to the LED
- Save and generate the configuration.

### 4. Build and Program

- Build the project using the ModusToolbox™ IDE.
- Connect your development board via USB.
- Program the board using the **"Program"** option.

### 5. Observe the LED

- Rotate the potentiometer knob.
- The LED's brightness should vary smoothly based on the potentiometer's position.

## Understanding the Code

The `main.c` file initializes the ADC and PWM components. In the main loop, it continuously reads the analog voltage from the potentiometer, maps this value to a suitable PWM duty cycle, and updates the PWM output to adjust the LED's brightness accordingly.

**Key Functions:**

- `Cy_SAR_StartConvert()`: Starts the ADC conversion.
- `Cy_SAR_IsEndConversion()`: Checks if the ADC conversion is complete.
- `Cy_SAR_GetResult16()`: Retrieves the ADC conversion result.
- `Cy_TCPWM_PWM_SetCompare0()`: Sets the PWM compare value to adjust the duty cycle.

## Customization

- **Dynamic Brightness Control**: Integrate additional input methods (e.g., buttons) to set predefined brightness levels.
- **Multiple LEDs**: Extend the project to control multiple LEDs with different PWM channels and potentiometers.
- **User Interface**: Implement UART communication to display the potentiometer value and corresponding PWM duty cycle.

## License

This project is open-source and available for educational and personal development use. Please refer to the `LICENSE` file for more information.

## Acknowledgments

- **Developed by**: Viral Patel
- **Tools Used**: ModusToolbox™, Infineon PSoC™ microcontrollers.

Feel free to contribute to this project by submitting issues or pull requests. Your feedback and improvements are welcome!

--- 
