// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/** State machine and pose estimation fuser for the Drive and Vision subsystems */
public class Chassis {
  private Drive robotDrive;
  private Vision robotVision;

  public Chassis(Drive drive, Vision vision) {
    robotDrive = drive;
    robotVision = vision;
  }

  /** Method that runs every loop cycle, called in robotPeriodic() */
  public void periodic() {
    for (int i = 0; i < robotVision.getNumberOfIOs(); i++) {
      if (robotVision.hasTarget(i)) {
        robotDrive.addVisionMeasurement(
            robotVision.getEstimatedRobotPose(i),
            robotVision.getLatestTimestampS(i),
            VecBuilder.fill(
                robotVision.getXStandardDeviation(i),
                robotVision.getYStandardDeviation(i),
                robotVision.getThetaStandardDeviation(i)));
      }

      Logger.recordOutput(
          "Chassis/Transform" + i,
          robotVision.getEstimatedRobotPose(i).minus(robotDrive.getOdometryPose()));
    }
  }
}
