# 🛩️ Pilot
The pilot's job is to drive the robot. While simple in theory, it is much more complex in practice. In order for successful operation of this robot, one must know the controls and the best strengths of its drivetrain.

## 🕹️ Controls
It is important to memorize controller bindings for optimal performance. A diagram of the controls for the pilot are below:

**Key**:
- *press*: You need to only tap and let go of the button for the action to occur
- *hold*: You must tap and hold the button for the action to continue. Letting go will interrupt the command and force its end state

![Pilot Controls](/docs/imgs/bindings-pilot.png)

**Description**:
- *Reset gyro*: Reset's the heading of the gyroscope to zero. This is used in determining the orientation of the robot. Note that whatever direction the **intake** is facing, will be the new "front" of the robot when this button is pushed.
  - You may need this if the orientation (translating forward and backward) is off due to an initialization fault.
- *Reset position*: Reset's the entire position of the robot to directly in front of the subwoofer in the centre, the intake facing the opposing alliance's wall.
  - Mostly used for debugging purposes
- *Auto heading*: Takes away rotational control from the pilot and will "lock" the **drive** to have the **shooter** always be facing the speaker
  - Note that this has not been tested. Please do not expect smooth or accurate behavior. Effective only within 3 meters of the speaker

## 🏎️ Chassis
As of 26/Sept. the drive control PIDs are largely untuned. This results in the swerve rotating slightly even if the pilot is demanding the swerve to translation straight. Current pilots will **need** to account for this. Precise operation of the swerve should remain largely unaffected.

The drive chassis is not fast. Cycles may be slower compared to other robots with better motors or module gear ratios. However, it is powerful. This robot is excellent for defence (as it is difficult to push) and good for *powering* through opposing defence. Synth will likely not outmaneuver other, lighter robots.