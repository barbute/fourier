// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive robotDrive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          robotDrive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * robotDrive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * robotDrive.getMaxLinearSpeedMetersPerSec(),
                  omega * robotDrive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? robotDrive.getRotation().plus(new Rotation2d(Math.PI))
                      : robotDrive.getRotation()));
        },
        robotDrive);
  }

  /**
   * Returns a commad to automatically align the robot to a desired heading without taking away
   * translation control
   */
  public static Command automaticHeading(
      Drive robotDrive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> omegaSupplier) {
    // Degress per second
    ProfiledPIDController thetaFeedback =
        switch (Constants.currentMode) {
          case REAL ->
              new ProfiledPIDController(
                  3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(300.0, 200.0));
          case SIM ->
              new ProfiledPIDController(
                  5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(300.0, 200.0));
          default ->
              new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        };

    thetaFeedback.setTolerance(0.2);
    thetaFeedback.enableContinuousInput(0.0, 360.0);

    return Commands.run(
        () -> {
          double omegaGoal =
              thetaFeedback.calculate(
                  robotDrive.getRotation().getDegrees(), omegaSupplier.get().getDegrees());

          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          robotDrive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * robotDrive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * robotDrive.getMaxLinearSpeedMetersPerSec(),
                  Math.toRadians(omegaGoal),
                  isFlipped
                      ? robotDrive.getRotation().plus(new Rotation2d(Math.PI))
                      : robotDrive.getRotation()));
        },
        robotDrive);
  }

  /** Returns a command to path find to a desire position */
  public static Command pathfindToPose(Drive robotDrive, Supplier<Pose2d> desiredPose) {
    return robotDrive.pathfindToPose(desiredPose);
  }

  /** Returns a command to set the velocity setpoint to 0 */
  public static Command stopDrive(Drive robotDrive) {
    return Commands.runOnce(() -> robotDrive.runVelocity(new ChassisSpeeds()), robotDrive);
  }
}
