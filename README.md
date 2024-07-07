STM32-Based Micro Drone Flight Controller
This project is a flight controller for a micro drone using an STM32 microcontroller, NRF24L, MPU6050, and BMP180. The system is still under development, focusing on PID tuning.

Features
STM32 Microcontroller: Core processing unit for the flight controller.
NRF24L Module: Provides 2.4GHz wireless communication.
MPU6050: Accelerometer and gyroscope for motion sensing.
BMP180: Barometric pressure sensor for altitude measurement.
PID Control: Ensures stable flight by adjusting motor speeds.
Components
STM32 Microcontroller
NRF24L01+ Module
MPU6050 Accelerometer and Gyroscope
BMP180 Barometric Pressure Sensor
Brushed Motors
Various peripheral components (e.g., resistors, capacitors)
Installation
Clone the repository:

git clone https://github.com/Emrecanbl/STM32-Based-Micro-Drone-Flight-Controller.git

Open the project:
Open the project in your preferred IDE (e.g., STM32CubeIDE).

Configure the environment:

Ensure that the project is set up with the correct STM32 microcontroller configuration.
Configure the I2C settings for the MPU6050 and BMP180.
Configure the SPI settings for the NRF24L module.
Compile and upload:
Compile the code and upload it to your STM32 microcontroller using a suitable programmer (e.g., ST-Link).

Usage
Connect the Sensors and Motors:

Connect the MPU6050 and BMP180 to the STM32 microcontroller via I2C.
Connect the NRF24L module to the STM32 microcontroller via SPI.
Connect the brushed motors to the appropriate GPIO pins on the STM32.
Power on the system:
Ensure that the STM32 microcontroller, sensors, and motors are properly powered.

Initialize the system:
The microcontroller will automatically start the communication with the sensors and process the data for flight control.

Tune the PID Controller:
Adjust the PID parameters to achieve stable flight.
![Sample](https://github.com/Emrecanbl/STM32-Based-Micro-Drone-Flight-Controller/blob/main/IMG_20240405_232716.jpg?raw=true)

Troubleshooting
No Communication: Check the connections and ensure the NRF24L modules and sensors are properly configured.
Sensor Errors: Verify the connections and calibration of the MPU6050 and BMP180.
Motor Issues: Check the connections and power supply for the motors.
Microcontroller Issues: Ensure the code is correctly compiled and uploaded to the STM32 microcontroller.
Contributing
Contributions are welcome! If you have any ideas, suggestions, or improvements, feel free to open an issue or submit a pull request.
![Sample](https://github.com/Emrecanbl/STM32-Based-Micro-Drone-Flight-Controller/blob/main/appearance.jpg?raw=true)

![Sample](https://github.com/Emrecanbl/STM32-Based-Micro-Drone-Flight-Controller/blob/main/appearance_2.jpg?raw=true)
License
This project is licensed under the MIT License. See the LICENSE file for more details.

Acknowledgments
Special thanks to the open-source community and the developers of the libraries and tools used in this project.




