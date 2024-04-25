# Embedded-remote-controlled-car

This project demonstrates the implementation of a remote-controlled car using Arduino and distance measurement sensors. The car can be controlled via an infrared (IR) remote control and is equipped with an ultrasonic distance sensor to detect obstacles and adjust its movement.

## Components Used

- Arduino board (Arduino Uno)
- Ultrasonic distance sensor (HC-SR04)
- IR receiver module
- Motor driver (for controlling the movement of motors)
- IR remote control

## Setup and Configuration

1. Connect the components to the Arduino board according to the pin configurations specified in the code.
2. Upload the provided Arduino sketch to the Arduino board.
3. Install the required libraries (Arduino_FreeRTOS, IRremote, etc.) on your Arduino IDE.

## Functionality

- **Distance Sensing Task**: Reads the distance from the ultrasonic sensor and updates the distance variable.
- **IR Sensor Task**: Receives IR commands from the remote control, decodes them, and performs corresponding actions such as adjusting speed and direction.
- **Stop Task**: Monitors the distance sensor and stops the car if an obstacle is detected within a critical distance.

## Usage

1. Point the IR remote control towards the IR receiver module.
2. Press the appropriate buttons on the remote control to control the movement of the car:
   - UP: Move forward
   - DOWN: Move backward
   - LEFT: Turn left
   - RIGHT: Turn right
   - SELECT: Stop and reset the car
3. The car will adjust its movement based on the distance readings from the ultrasonic sensor to avoid collisions with obstacles.

## Notes

- Ensure that the motor driver and power supply are capable of handling the current requirements of the motors used in the car.
- Adjust the critical distance and motor speed settings in the code as needed.

## Sami Almousa
