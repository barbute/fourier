// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.DoubleSupplier;

/** Defines the logic for converting driver controller inputs to useable data */
public class TeleoperatedController {
  private final double DEADBAND = 0.1;

  /** Variable to square the inputs of the control, useful for more "fine" control of the robot */
  private final boolean SQUARE_INPUTS = true;

  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;
  private DoubleSupplier thetaSupplier;

  /**
   * Binds the suppliers from the controller to the suppliers of this class to be used. See WPILib
   * coordinates for more info.
   *
   * @param xSupplier Relative forward-backward input
   * @param ySupplier Relative left-right input
   * @param thetaSupplier Relative rotational input
   */
  public TeleoperatedController(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.thetaSupplier = thetaSupplier;
  }

  /**
   * Calculates the desired ChassisSpeeds from driver input suppliers (setup in the constructor)
   *
   * @param robotHeading The current robot heading
   * @param currentRobotRelativeSpeeds The robot's current speed state (relative to its front)
   * @return The computed speeds
   */
  public ChassisSpeeds computeChassisSpeeds(
      Rotation2d robotHeading,
      ChassisSpeeds currentRobotRelativeSpeeds,
      double maxLinearSpeedMetersPerSec,
      double maxAngularSpeedRadPerSec) {
    // Apply deadbands
    double linearMagnitude =
        MathUtil.applyDeadband(
            Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    double theta = MathUtil.applyDeadband(thetaSupplier.getAsDouble(), DEADBAND);

    if (SQUARE_INPUTS) {
      linearMagnitude = linearMagnitude * linearMagnitude;
      theta = Math.copySign(theta * theta, theta);
    }

    // New velocity represented as a vector
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection) // Take the initial point and direction
            .transformBy(
                new Transform2d(
                    linearMagnitude,
                    0.0,
                    new Rotation2d())) // Calculate the "end point" of the vector (magnitude)
            .getTranslation(); // Store as a vector

    // The heading will naturally be flipped relative to the field (when ChassisSpeeds is being
    // computed it does not take into account whether or not your robot heading is flipped relative
    // to the rest of the field)
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    ChassisSpeeds computedSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * maxLinearSpeedMetersPerSec,
            linearVelocity.getY() * maxLinearSpeedMetersPerSec,
            theta * maxAngularSpeedRadPerSec,
            isFlipped ? robotHeading.plus(new Rotation2d(Math.PI)) : robotHeading);

    return computedSpeeds;
  }
}
