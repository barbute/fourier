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

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2
  private static final double DRIVE_GEAR_RATIO = 6.75 / 1.0;
  private static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;

  private final CANSparkMax driveMotor;
  private final CANSparkMax azimuthMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder azimuthRelativeEncoder;
  private final AnalogInput azimuthAbsoluteEncoder;

  private final boolean isAzimuthMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveMotor = new CANSparkMax(1, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(2, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new AnalogInput(0);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 1:
        driveMotor = new CANSparkMax(3, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(4, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new AnalogInput(1);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 2:
        driveMotor = new CANSparkMax(5, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(6, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new AnalogInput(2);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      case 3:
        driveMotor = new CANSparkMax(7, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(8, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new AnalogInput(3);
        absoluteEncoderOffset = new Rotation2d(0.0); // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveMotor.restoreFactoryDefaults();
    azimuthMotor.restoreFactoryDefaults();

    driveMotor.setCANTimeout(250);
    azimuthMotor.setCANTimeout(250);

    driveEncoder = driveMotor.getEncoder();
    azimuthRelativeEncoder = azimuthMotor.getEncoder();

    azimuthMotor.setInverted(isAzimuthMotorInverted);
    driveMotor.setSmartCurrentLimit(40);
    azimuthMotor.setSmartCurrentLimit(30);
    driveMotor.enableVoltageCompensation(12.0);
    azimuthMotor.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    azimuthRelativeEncoder.setPosition(0.0);
    azimuthRelativeEncoder.setMeasurementPeriod(10);
    azimuthRelativeEncoder.setAverageDepth(2);

    driveMotor.setCANTimeout(0);
    azimuthMotor.setCANTimeout(0);

    driveMotor.burnFlash();
    azimuthMotor.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};

    inputs.azimuthAbsolutePosition =
        new Rotation2d(
                azimuthAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    inputs.azimuthPosition =
        Rotation2d.fromRotations(azimuthRelativeEncoder.getPosition() / AZIMUTH_GEAR_RATIO);
    inputs.azimuthVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(azimuthRelativeEncoder.getVelocity())
            / AZIMUTH_GEAR_RATIO;
    inputs.azimuthAppliedVolts = azimuthMotor.getAppliedOutput() * azimuthMotor.getBusVoltage();
    inputs.azimuthCurrentAmps = new double[] {azimuthMotor.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  @Override
  public void setAzimuthVoltage(double volts) {
    azimuthMotor.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setAzimuthBrakeMode(boolean enable) {
    azimuthMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
