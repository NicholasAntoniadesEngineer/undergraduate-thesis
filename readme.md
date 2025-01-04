# Servomotor Controller Board

## Abstract

The purpose of this study is to design a servomotor controller board that replicates the functionality of more expensive servomotor devices but at a significantly lower cost. The controller board offers advanced control features, including serial communication hardware and temperature and current sensors, all managed by an on-board microcontroller. 

The system employs a master-slave architecture, where a master device controls multiple slave controllers, communicating either a change in position or a request for information. The on-board power supply supports a wide current range, making the controller compatible with various servomotors.

All components, except for the buck converter, were successfully tested. It was discovered that the pad spacing between the IC pins on the PCB design did not include solder mask, which caused shorts between power and ground pins. This issue will be addressed in future iterations by ensuring proper solder mask application.

Despite this issue, the system’s overall cost is substantially lower than commercially available servomotor controllers. While all modules functioned during testing, a revised PCB design is necessary to build a fully operational prototype.

## Features

- **Serial Communication**: Serial communication hardware for control and feedback.
- **Sensor Integration**: On-board temperature and current sensors for real-time monitoring.
- **Wide Current Range**: Compatible with a large range of servomotors.
- **Master-Slave Architecture**: Centralized control of multiple slave controllers.
- **Cost-Effective**: Lower cost compared to standard servomotor controllers on the market.

## Issues and Future Work

- **PCB Design**: A design flaw with the solder mask between IC pins resulted in short circuits when power was applied. Future designs will address this by improving the solder mask application between pads.
- **Buck Converter**: The buck converter was not successfully tested and requires further debugging.
- **Next Steps**: A second PCB design is necessary to produce a functional prototype.

## Repository Contents

This repository contains all design files, code, and documentation related to the servomotor controller board project, including:

- PCB design files
- Microcontroller firmware
- Test scripts for sensor and communication modules
- Documentation and reports

