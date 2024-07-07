STM32-Based Micro Drone Flight Controller
This project is the flight controller for a micro drone, utilizing an STM32 microcontroller and several peripheral sensors and components. It is designed to work with the NRF24L transceiver module for communication and includes functionality for sensor integration and motor control.

Features
STM32 Microcontroller: Provides the core processing capabilities for the flight controller.
NRF24L Transceiver: Ensures reliable wireless communication with the drone controller.
MPU6050: Integrates a 3-axis gyroscope and accelerometer for flight stabilization.
BMP180: Barometric pressure sensor for altitude measurement.
Brushed Motors: Provides thrust and control for the drone's flight.
PID Control: Implements a PID control algorithm for stable flight (currently under tuning).
Components
STM32 Microcontroller
NRF24L Transceiver Module (connected via SPI)
MPU6050 Accelerometer and Gyroscope
BMP180 Barometric Pressure Sensor
Brushed Motors
Installation
Clone the repository:

sh
Kodu kopyala
git clone https://github.com/Emrecanbl/STM32-Based-Micro-Drone-Flight-Controller.git
Open the project:
Open the project in your preferred IDE (e.g., STM32CubeIDE).

Compile and upload:
Compile the code and upload it to your STM32 microcontroller using a suitable programmer (e.g., ST-Link).
![Sample](https://github.com/Emrecanbl/STM32-Based-Micro-Drone-Flight-Controller/blob/main/IMG_20240405_232716.jpg?raw=true)
Usage
Power on the drone:
Ensure the drone is powered and the sensors and motors are connected properly.

Pairing:
The drone will automatically attempt to pair with the NRF24L-based controller.

Flight:
Use the controller to pilot the drone. The flight controller integrates sensor data and executes the PID control algorithm for stable flight.

PID Tuning
PID tuning is crucial for achieving stable flight. Adjust the PID coefficients in the code to optimize the drone's response to control inputs.
![Sample](https://github.com/Emrecanbl/STM32-Based-Micro-Drone-Flight-Controller/blob/main/appearance.jpg?raw=true)
Troubleshooting
Connection Issues: Ensure that the NRF24L modules on both the drone and the controller are powered and within range.
Sensor Problems: Verify the connections and calibrations of the MPU6050 and BMP180 sensors.
Motor Control Issues: Check the connections to the brushed motors and ensure they are receiving the correct signals from the microcontroller.
PID Tuning: If the drone is unstable, adjust the PID coefficients to better match the drone's dynamics.
Contributing
Contributions are welcome! If you have any ideas, suggestions, or improvements, feel free to open an issue or submit a pull request.
![Sample](https://github.com/Emrecanbl/STM32-Based-Micro-Drone-Flight-Controller/blob/main/appearance_2.jpg?raw=true)
License
This project is licensed under the MIT License. See the LICENSE file for more details.

Acknowledgments
Special thanks to the open-source community and the developers of the libraries and tools used in this project.


