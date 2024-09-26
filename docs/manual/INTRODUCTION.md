# 🤖 Operator's Manual
In order to successfully operate this robot, it is important to know its mechanisms and how leverage the robot's strengths and weaknesses.

Reference the pages below for specific bindings:
- 🛩️ [Pilot Information](/docs/manual/PILOT.md)
- 📱 [ Copilot Information](/docs/manual/COPILOT.md)

## 🦾 Mechanisms
To briefly summarize, Synth is composed of three primary structures: **drive**, **shooter**, **rollers**. the *pilot* is responsible for operating the **drive** while the *copilot* is responsible for operating the **shooter** and **rollers**.

### 🏎️ Drive
The drive is an SDS MK4i chassis powered by two REV NEO motors for driving power and azimuth rotation. For a more in-depth explanation of what *swerve* is see [this](https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html).

> 📌 **NOTE** As of 26/Sept. the drive control PIDs are largely untuned. This results in the swerve rotating slightly even if the pilot is demanding the swerve to translation straight. Current pilots will **need** to account for this. Precise operation of the swerve should remain largely unaffected.

What the robot lacks in speed it makes up for in power. The drive is powerful and is able to defend with ease. Odds are this robot *will* win a pushing battle against other robots. However, the maximum attainable velocity and acceleration is rather low, it is important to factor this in when making endgame cycles.

### 🔫 Shooter
The shooter is composed of a barrel situated on a **pivot** powered by a REV NEO *leadscrew* mechanism, and **flywheels** powered by two CTRE Falcon 500s. Note that a *leadscrew* is a mechanism that converts rotational power into linear motion; While this was intended to result in much more stable control of the machine, it ultimately resulted in a loss of precision and speed when adjusting the angle of the shooter.

Despite the **pivot** being quite sub-optimal, it is able to acomplish the objective of the game: scoring notes. The **pivot** is effective within 0.25 degrees of tolerance.

> 📌 **NOTE** The **pivot** uses an absolute encoder to always reset its position. It may occasionally fail to startup correctly; Do not worry though! The **pivot** will still be effective, but it may require a bit more precesion on the pilot's part when using fixed setpoints.

The **flywheels** require at least half a second to get up to the desired speed.

Note that haptic feedback will be provided to the copilot when the shooter is at its desired state.

### 🎱 Rollers
The rollers is composed of an **intake** and an **indexer**, each being individually powered by a REV NEO motor. The **intake** acts to grab a note and bring it within the robot's frame-perimeter. The **indexer** will stop the note once far enough inside of the barrel, and fire the note when desired.

The **indexer** has a beam-break sensor to detect when a note is in the barrel. This will stop the note at the same place every time. However, there is a chance it will fail, in which case the note may go further than anticipated.
