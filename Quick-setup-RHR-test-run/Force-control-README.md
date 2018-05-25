## Rough force control

How to send a command
An interesting feature in the TakkTile and SF code is force control, the motors can be commanded to a force and a control loop in the code tries to follow that command. To get out of this mode, just send another type of command, like those shown above.

### Assuming code is still running

    rostopic pub /your_gripper /command_motor_force reflex_msgs/ForceCommand "f1: 200.0
    f2: 200.0 f3: 200.0 preshape: 0.0"

NOTE: Whether or not tactile stops are enabled, a finger in this state will NOT halt its motion upon contact, because the idea of force control is to provide relatively constant pressure while in contact. The finger will loosen when overloaded, leading to oscillations if the commanded force is too close to the overload threshold. More details about the overload threshold can be found in the Safety Overloads section of the open and close fingers tutorial.

NOTE: The control code attempts to keep the force on the actuating motor constant, which means that actual applied force will vary depending on where along the finger the force is applied. This is because the finger acts as a variable lever arm. It could perhaps be compensated for using tactile data to locate the object on the finger.
