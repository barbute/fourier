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

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of ModuleIO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final double DRIVE_GEAR_RATIO = 6.75 / 1.0;
  private final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), DRIVE_GEAR_RATIO, 0.025);
  private DCMotorSim azimuthSim = new DCMotorSim(DCMotor.getNEO(1), AZIMUTH_GEAR_RATIO, 0.004);

  private final Rotation2d azimuthAbsoluteInitPosition =
      new Rotation2d(Math.random() * 2.0 * Math.PI);

  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(LOOP_PERIOD_SECS);
    azimuthSim.update(LOOP_PERIOD_SECS);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    inputs.driveTemperatureCelsius = new double[] {0.0};

    inputs.azimuthAbsolutePosition =
        new Rotation2d(azimuthSim.getAngularPositionRad()).plus(azimuthAbsoluteInitPosition);
    inputs.azimuthPosition = new Rotation2d(azimuthSim.getAngularPositionRad());
    inputs.azimuthVelocityRadPerSec = azimuthSim.getAngularVelocityRadPerSec();
    inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    inputs.azimuthCurrentAmps = new double[] {Math.abs(azimuthSim.getCurrentDrawAmps())};
    inputs.azimuthTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setAzimuthVoltage(double volts) {
    azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    azimuthSim.setInputVoltage(azimuthAppliedVolts);
  }
}
