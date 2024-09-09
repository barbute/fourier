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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** ModuleIO implementation for SparkMax drive motor controller, SparkMax turn motor controller */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2
  private static final double DRIVE_GEAR_RATIO = 6.75 / 1.0;
  private static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;

  private final CANSparkMax driveMotor;
  private final CANSparkMax azimuthMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder azimuthRelativeEncoder;
  private final CANcoder azimuthAbsoluteEncoder;

  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveMotor = new CANSparkMax(11, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(21, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new CANcoder(31, "CTREBUS");

        absoluteEncoderOffset = Rotation2d.fromRotations(0.0);
        break;
      case 1:
        driveMotor = new CANSparkMax(12, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(22, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new CANcoder(32, "CTREBUS");

        absoluteEncoderOffset = Rotation2d.fromRotations(0.0);
        break;
      case 2:
        driveMotor = new CANSparkMax(13, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(23, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new CANcoder(33, "CTREBUS");

        absoluteEncoderOffset = Rotation2d.fromRotations(0.0);
        break;
      case 3:
        driveMotor = new CANSparkMax(14, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(24, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new CANcoder(34, "CTREBUS");

        absoluteEncoderOffset = Rotation2d.fromRotations(0.0);
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

    driveMotor.setInverted(false);
    azimuthMotor.setInverted(true);

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

    driveMotor.setIdleMode(IdleMode.kBrake);
    azimuthMotor.setIdleMode(IdleMode.kCoast);

    driveMotor.setCANTimeout(0);
    azimuthMotor.setCANTimeout(0);

    driveMotor.burnFlash();
    azimuthMotor.burnFlash();

    azimuthAbsoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
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
        new Rotation2d(azimuthAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())
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
