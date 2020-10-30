# TypingAid

Controller that can lift a human's arm to aid with typing.

Two hardware systems are targeted: Controller code under `franka_typing_aid` is designated for the Franka 7-DOF robot arm. Code under `anydrive_typing_aid` can be executed with an ANYdrive-based setup.

## Setup `franka_typing_aid`

Requirements:

Install ROS melodic. Install the following package(s):

```bash
sudo apt install ros-melodic-libfranka
```
