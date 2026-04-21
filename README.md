Go Baby Go Landers Project
Overview
The Go Baby Go Lander project is part of an initiative to design and develop a series of adaptive vehicles aimed at providing adaptive play experiences for children with mobility challenges. 
The primary goal is to modify existing ride-on cars to allow for easy control and customized functionality, empowering children with limited mobility to explore and engage with their environment more independently.

This repository contains the code and hardware design for the various ride-on modifications, including motor control, joystick interface, ramp control, and potential integration of advanced features such as parallel drive systems and adaptive steering mechanisms.

Features
Arcade-Style Drive System: The project uses joystick-controlled arcade-style drive logic, allowing the vehicle to be controlled with precise forward/backward and turning inputs.

GoBabyGo - ESP32 Lander Version
Adaptive Mobility for Kids

This repository contains the control software for a modified 24V ride-on car using an ESP32 and Cytron Motor Drivers.

🚀 Key Features in Version 3 (Stable)
Safety Watchdog: Instant motor cutoff when the joystick is in the deadzone.

Soft Ramping: Gentle acceleration to protect plastic gearboxes and prevent "jerky" starts for the rider.

Dynamic Calibration: Automatically sets the joystick center point at boot to account for sensor drift.

Arcade Drive Logic: Intuitive one-handed steering (X-axis for turning, Y-axis for forward/reverse).

🛠 Hardware Configuration
Controller: ESP32 Dev Module

Motor Driver: Cytron MD10C or MD13S (PWM-DIR Mode)

Power: 24V Battery System with Buck Converter for logic power.

📌 File Guide
GoBabyGo_Main.ino: The current stable version (Version 3). Use this for all new builds.

Archive/: (Optional) Folder where I moved the old, jerky versions for reference.

⚙️ How to Tune
To adjust the car's performance, modify these constants at the top of the code:

powerCutFactor: Change from 0.1 (slow) to 1.0 (full speed).

rampStep: Lower values (e.g., 2) make acceleration smoother; higher values (e.g., 10) make it snappier.

deadzone: Increase if the car "creeps" when nobody is touching the joystick.

There are additional options to tune direction.  For example, power can be multiplied by a negative for direction, and joystick pins can be swapped if needed. 

Advanced Features (Future Plans):
4-Wheel Drive Ride Ons: Modify the drivetrain to support 4-wheel drive, increasing stability and control.

Adaptive Steering: Implement a parallelogram-style steering mechanism, potentially using large servos or linear actuators for more precise control.

Contributing
We welcome contributions! If you'd like to help improve the project, please fork the repository, make your changes, and submit a pull request. When submitting changes, please ensure that the code is well-documented and follows the existing coding standards.

Issues:
If you encounter any issues or have suggestions for improvements, please open an issue on GitHub. We aim to address issues promptly and work together to improve the project.

License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgements
Special thanks to the open-source community and contributors who have helped make adaptive play more accessible for children. Your support and feedback are invaluable.

