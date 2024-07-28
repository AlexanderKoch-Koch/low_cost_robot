# Low-Cost Robot Arm
Join the waitlist here to get all the parts in one package: https://tau-robotics.com/robots

This repository contains the files to build and control a low-cost robot arm that costs about $250. You can also build a second robot arm (the leader arm) to control the other arm (the follower arm) that costs about $180, for a total of $430. The design of the leader is inspired by the [GELLO project](https://github.com/wuphilipp/gello_mechanical) but is simpler to build. Such a robot arm is well suited for [robot learning](https://x.com/alexkoch_ai/status/1756500716854841835?s=20). Two of those arms are also capable of [folding clothes](https://x.com/alexkoch_ai/status/1772750496174149708?s=20).

This robot arm uses Dynamixel XL430 and Dynamixel XL330 servo motors. The XL430 motors are almost twice as strong and are used for the first two joints.
The XL330 motors are weaker but weigh only 18g each. This makes the arm very lightweight and fast.
Dynamixel sells the U2D2 adapter to connect the servos to a computer. However, this is very expensive and the latency is very high. This build uses another cheaper adapter board instead.
The robot arm can be controlled with the Dynamixel SDK: ```pip install dynamixel-sdk```

![Robot Arm](./pictures/robot_portait.jpg)

## Follower Arm

### Required Materials

| Part                          | Cost | Buying link                                    | Specs |
|-------------------------------|------|------------------------------------------------| --- |
| 2x Dynamixel XL430-W250       | $100 | https://www.robotis.us/dynamixel-xl430-w250-t/ | https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ |
| 4x Dynamixel XL330-M288       | $96  | https://www.robotis.us/dynamixel-xl330-m288-t/ | https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/|
| XL330 Idler Wheel             | $10  | https://www.robotis.us/fpx330-h101-4pcs-set/   | **Note**: pack of four; three needed for  longer version pictured above (with elbow-to-wrist extension), two needed for shorter version shown in the [assembly video](https://youtu.be/RckrXOEoWrk)|
| XL430 Idler Wheel             | $7   | https://www.robotis.us/hn11-i101-set/          | |
| Waveshare Serial Bus Servo Driver Board | $10  | https://a.co/d/7C3RUYU                         | |
| Voltage Reducer               | $10   | https://a.co/d/cy02ADW                         | **Note**: pack of six, only one needed per follower arm |
| 12V Power Supply              | $12  | https://a.co/d/40o8uMN                         | |
| Table Clamp                   | $6   | https://a.co/d/4KEiYdV                         | |
| Wires                         | $7   | https://a.co/d/hQfk2cb                         | |
| Total                         | $258 |                                                | |

There is usually a 10% discount code for the Robotis shop. It might also help to add some grip tape to the gripper (e.g. https://a.co/d/dW7BnEN). A USB-C cable is necessary to connect the servo driver board to a computer.

![follower](./pictures/follower_arm.png)

### Assembly

Video of the assembly: https://youtu.be/RckrXOEoWrk

1. Print all parts with a 3D printer
   1. The STL files are in `hardware/follower/stl`
   2. The parts are designed to be easy to print; only the moving part of the gripper needs supports
2. Scanning motors
   1. Connect the driver board to a computer (should work with Linux and MacOS)
   2. Figure out the device name (e.g. tty.usbmodem57380045631 for MacOS): ```ls /dev/tty.*```
   3. Scan each motor individually with [Dynamixel Wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
      1. Set the baudrate to 1M for all motors
      2. Set the servo IDs to 1 for the shoulder to 5 (6 if using the elbow-to-wrist extension) for the gripper servo
3. Assembly
   1. Assemble the arm without the base
      1. Make sure that the servos are fixed in the same position as in the CAD
      2. The servo horn should be in the default position when screwed in
   2. Solder wires onto voltage reducer; input should be connected to female connectors and the output to male connectors
   3. Screw the voltage reducer and the servo driver board onto the base
   4. Screw the base onto the arm
   5. Connect D, V, and G ports on the driver board to the shoulder rotation servo
   6. Connect the shoulder rotation servo to the shoulder lift servo
   7. Connect the input for the voltage reducer to V and G ports on the driver board
   8. Connect the output of the voltage reducer and the remaining D port of the driver board to the elbow servo
   9. Connect the driver board to the power supply
   10. Connect to an XL330 servo and view the input voltage on Dynamixel Wizard, then adjust the screw on the voltage reducer until the input voltage is 5V

## Leader Arm

### Required Materials

| Part                          | Cost | Buying link | Specs |
|-------------------------------|------| --- | --- |
| 6x Dynamixel XL330-M077       | $144 |  https://www.robotis.us/dynamixel-xl330-m077-t/ | https://emanual.robotis.com/docs/en/dxl/x/xl330-m077/|
| XL330 Frame | $7   | https://www.robotis.us/fpx330-s101-4pcs-set/ | |
| XL330 Idler Wheel             | $10  | https://www.robotis.us/fpx330-h101-4pcs-set/   | **Note**: pack of four; three needed for longer version (with elbow-to-wrist extension), two needed for shorter version pictured below |
| Waveshare Serial Bus Servo Driver Board | $10  | https://a.co/d/7C3RUYU | |
| 5V Power Supply               | $6   | https://a.co/d/5u90NVp | |
| Table Clamp                   | $6   | https://a.co/d/4KEiYdV | |
| Total                        | $183 | | |

![leader](./pictures/leader_arm.png)

### Assembly

The assembly of the leader arm is simpler since all motors use 5V. The gripper is replace by a handle and a trigger. During use, a small torque can be applied to the trigger so that it opens by default. The GELLO design uses a spring for this purpose but it is much more difficult to assemble.
The `teleoperation.py` script can be used to test the arms. However, the device names might have to be adjusted.

## Simulation
A basic simulation environment in MuJoCo is available by running `simulation.py`.
