# 📱 Copilot
The copilot's job is the operate the superstructure of the robot - which includes the **shooter**, **climb**, **intake**, and **indexer** subsystems. This process has been automated to a degree, though one must still know the controls and capabilities of the superstructure.

## 🕹️ Controls
It is important to memorize controller bindings for optimal performance. Note that when attempting to *aim*, *feed*, or *score* the copilot's controller will rumble when the **shooter** is at its desired goal. A diagram of the controls for the pilot are below:

**Key**:
- *press*: You need to only tap and let go of the button for the action to occur
- *bumper*: The button above the trigger
- *trigger*: A button that can take a percent output of how mcuh it is being pressed

![Copilot Controls](/docs/imgs/bindings-copilot.png)

**Description**:
- *Fire-amp*: Runs the **indexer** and **intake** at a speed suitable for launching a note out of the barrel towards the amp.
  - Used only when attempting to score into the amp
- *Fire-speaker*: Runs the **indexer** and **intake** at a speed suitable for launching a note out of the barrel towards the speaker.
  - Used in all other cases when attempting to eject a note out of the barrel
- *Stow*: Runs the **indexer** until it detects a note inside of the barrel. The **intake** will also stop when the note has been detected. The **shooter** will also pivot to an optimal intaking angle.
- *Outtake*: Runs the **indexer** and **intake** outwards to spit a note out of in the **intake**.
  - May be useful for clearing jams in the barrel or transition between **intake** and **indexer**
- *Aim-subwoofer*: Runs the **shooter** to a pre-determined setpoint optimal for firing into the speaker when the robot is directly in front of the subwoofer.
  - Note that the robot's bumpers **must** be touching the front-face of the subwoofer in order for this setpoint to fire the note in correctly
- *Aim-amp*: Runs the **shooter** to a pre-determined setpoint optimal for firing into the amp when the robot is directly front-and-center of the amp hole.
  - For this setpoint to work, the robot must be fairly center with the opening and should be touching the amp
- *Feeder*: Runs the **shooter** to a pre-determined setpoint optimal for launching notes across the field.
  - The robot should be past the opposing alliance's wing line and the shooter should be angled to the amp-side corner of the field
- *Aim-lock*: Runs the **shooter** to constantly be aiming at the speaker - adjusting its angle as its computed distance changes.
  - Note that as of 26/Sept. this has not been tested. It may not work as intended or be very accurate. Should it work, it will only be effective up to three meters away from the speaker.
> 📌 **NOTE** As of 26/Sept. there are no soft limits for the shooter pivot or climb chassis when running with voltage. **DO NOT** push these mechanisms past their physical limits. I will remove this warning when the issue has been resolved.
- *Shooter-up*: Runs the **shooter** upwards using only voltage. If PIDs are completely broken, this should be used as a fallback.
- *Shooter-down*: Runs the **shooter** downards using only voltage. If PIDs are completely broken, this should be used as a fallback.
- *Climb-up*: Runs the **climb** elevators out of the robot.
- *Climb-down*: Runs the **climb** elevators down into the robot.