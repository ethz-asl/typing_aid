# Typing-8

Controller for the Franka Arm that can lift a human's arm to aid with typing.

## Setup

Requirements:

Install ROS melodic. Install the following package(s):

```bash
sudo apt install ros-melodic-libfranka
```

## Experiments

### Can human arms be lifted?

**Questions to be answered**:

- Can the Franka arm lift a passive human arm?
- How fast is the lifting process?
- What values do we need to set the Franka arm's parameters (collision behavior, joint impedance, cartesian impedance) to for this to work?

**Proposed setup**: Franka arm in home position, string (about 30 cm) attached to the hand.

**Proposed procedure**:

1. Start Franka arm and set to autonomous mode.
1. Launch the program: `rosrun typing-8 controller $FRANKA-IP` (not sure if the variable name `$FRANKA-IP` is correct; adapt if necessary).
1. Hit enter. The arm will move to the home position.
1. Hit enter again to start gravity compensation mode.
1. Hold on to the string which is attached to the hand of the Franka arm. Keep the e-stop close by in case something unexpected happens.
1. Publish the trigger message from another terminal: `rostopic pub -1 /lifting_trigger std_msgs/Empty "{}"`. Observe what happens, i.e. whether the robot arm can lift a human arm.

**Parameters to play with**:

To avoid the arm going into collision mode when supposed to lift the human arm, the following parameters may need to be adapted:

- Collision behavior, defined in `src/examples_common.cpp`, documented [here](https://github.com/frankaemika/libfranka/blob/06ad8dcf5706f00663b6fd6351734096cea9c2d0/include/franka/robot.h#L483).
- Joint impedance, defined in `src/examples_common.cpp`, documented [here](https://github.com/frankaemika/libfranka/blob/06ad8dcf5706f00663b6fd6351734096cea9c2d0/include/franka/robot.h#L533).
- Cartesian impedance, defined in `src/examples_common.cpp`, documented [here](https://github.com/frankaemika/libfranka/blob/06ad8dcf5706f00663b6fd6351734096cea9c2d0/include/franka/robot.h#L547).
