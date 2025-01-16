# Servo Motor Controller Board (2018)

This project report documents the creation of a cost-effective servomotor controller board, aimed at providing functionality comparable to high-end commercial controllers but at a significantly reduced cost. 

## Project Overview

Objective: To design a servomotor controller that offers advanced control (position, current, temperature) with serial communication capabilities.

## Design & Development

### Hardware
Utilized an STM32F051C6 microcontroller, various sensors (current, temperature), a buck converter for power management, and RS485 for communication. 

### Software
Code written in C using Atollic TrueSTUDIO for managing ADC for sensor readings, PWM for motor control, and USART for communication between master and slave units.

## Testing & Results

### Component Testing
Each component was tested individually. Sensors, voltage regulators, and communication chips worked as expected. 

### PCB Design & Issues
The PCB design was flawed due to missing solder mask between IC pins, leading to shorts when powered. This issue was identified during assembly, highlighting the need for precise PCB design.

## Key Findings
- Cost: The total cost for components was R290.36, excluding PCB manufacturing and assembly.
- Functionality: While the PCB did not function due to design errors, individual component tests proved the concept viable.
- Lessons Learned: Importance of checking component availability in desired packages, accurate PCB footprint design, and ensuring proper isolation between IC pins.

## Recommendations
- Improve PCB footprint design accuracy.
- Ensure component package compatibility before finalizing designs.
- Use indicator LEDs for easier debugging and test points for component testing on the PCB.

## Conclusion
The project was successful in terms of conceptual design and component functionality but failed at the PCB assembly stage due to manufacturing errors. This serves as a learning opportunity for future iterations in electronic design projects.


