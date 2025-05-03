# TriOmniDrive

<p align="justify">
This repository contains the code to implement inverse kinematics on a three-wheeled omnidirectional robot.</p> 

<p align="justify">
With this approach, velocity commands can be sent in the form of a ROS2 <code>Twist</code> message, specifying the desired linear velocities <code>Vx</code>, <code>Vy</code>, and angular velocity <code>ω</code>, which are then converted into individual wheel angular velocities.
</p>

<p align="center"> 
  <img src="https://github.com/user-attachments/assets/1a640358-42fa-431d-b2d3-d7f69b7eaa03" alt="Inverse kinematics on a three-wheeled omnidirectional robot demo"/> 
</p>

<p align="justify">
This is an activity for the Mobile Ground Robots module in the undergraduate course <strong>"Intelligent Robotics Implementation."</strong>
</p>

<p align="justify"> To operate the physical robot, you will need to have <a href="https://docs.ros.org/en/humble/Installation.html" target="_blank">ROS2 Humble</a> installed on your system. This project was developed and tested on Ubuntu Linux 22.04 (Jammy Jellyfish). </p>

## Kinematics Overview

<p align="justify">
The file <code>scripts/omni3_inverse_kinematics.py</code> provides an overview of the inverse kinematics model used for the robot. It demonstrates how robot-level velocities are converted into wheel speeds using matrix operations. You can run this script for testing using:
</p>

```bash
python3 omni3_inverse_kinematics.py
```

## Use of the Omnidirectional Robot

<ol>
  <li>
    Install gRPC Tools in Python from 
    <a href="https://pypi.org/project/grpcio-tools/" target="_blank">here</a>:
    <p><code>pip install grpcio grpcio-tools</code></p>
  </li>
  <li>
    Clone this repository:
    <p><code>git clone https://github.com/fectec/TriOmniDrive.git</code></p>
  </li>
  <li>
    <p align="justify">
        Join the robot's onboard Raspberry Pi Wi-Fi network using the provided SSID and password. Once connected, the Raspberry Pi can be accessed at the IP address <code>192.168.2.103</code>.
    </p>
  </li>
  <li>
    <p align="justify">
      Transfer the file <code>scripts/RPIMotorService.py</code>, which implements the gRPC server running on the Raspberry Pi. This server allows the robot to receive velocity instructions remotely over the network and translate them into low-level motor control using GPIO and encoder feedback. The script initializes the robot's physical parameters, sets up encoder interrupts, and runs a PID control loop to achieve the desired wheel speeds. Use the following command from your Ubuntu system:
    </p>
    <p>
    <code>scp /path/to/TriOmniDrive/scripts/RPIMotorService.py pi@192.168.2.103:/home/pi/omni_ros2/src/oav_utils/scripts</code>
    </p>
  </li>
  <li>
    <p align="justify">
      After transferring the file, connect to the Raspberry Pi using SSH, navigate to the target directory, verify that the file is present, and execute it:
    </p>
    <p>
    <code>ssh pi@192.168.2.103 # Password is 1234</code><br>
    <code>cd /home/pi/omni_ros2/src/oav_utils/scripts</code><br>
    <code>python3 RPIMotorService.py</code></p>
    <p align="justify">
      At this point, the robot's motor control server will be running and waiting for velocity commands via gRPC.
    </p>
  </li>
  <li> 
    <p align="justify"> 
      Navigate to the <code>ros2_ws</code> directory in the cloned repository and build the workspace using <code>colcon</code>: 
    </p> 
    <code>colcon build --symlink-install</code><br>
    <code>source install/setup.bash</code>  
    </p> 
  </li> 
  <li> 
    <p align="justify">
      Connect an Xbox One Elite Series controller (or any joystick). You may customize the joystick config files in:
    </p>
    <ul>
      <li><code>triomnidrive_control/config/joystick_config.yaml</code></li>
      <li><code>triomnidrive_control/config/joystick_teleop.yaml</code></li>
    </ul>
  <li>
    <p align="justify">
      Run the joystick teleoperation launch file:
    </p>
    <code>ros2 launch triomnidrive_control joystick_teleop.launch.py</code>
    <p align="justify"> 
      This will start the gRPC client node, joystick driver, and teleoperation node to control the robot. </p> 
  </li> 
</ol>
