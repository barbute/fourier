// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** ClimbIO implementation for SparkMax motor controller (NEO) */
public class ClimbIOSparkMax implements ClimbIO {
  private CANSparkMax leftMotor = new CANSparkMax(61, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(62, MotorType.kBrushless);

  private DutyCycleEncoder leftAbsoluteEncoder = new DutyCycleEncoder(1);
  private DutyCycleEncoder rightAbsoluteEncoder = new DutyCycleEncoder(4);

  // Left upper limit: -2.594 rotations
  // Left lower limit:

  // 2 1/4 in per rotation

  private Rotation2d leftPositionOffset = Rotation2d.fromDegrees(0.0);
  private Rotation2d rightPositionOffset = Rotation2d.fromDegrees(0.0);

  private double leftLinearPositionOffsetMeters = 0.0; // 0.0574;
  private double rightLinearPositionOffsetMeters = 0.0;

  public ClimbIOSparkMax() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    leftMotor.clearFaults();
    rightMotor.clearFaults();

    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    leftMotor.setSmartCurrentLimit(40);
    leftMotor.enableVoltageCompensation(12.0);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit(40);
    rightMotor.enableVoltageCompensation(12.0);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setInverted(true);
    rightMotor.setInverted(true);

    leftAbsoluteEncoder.setDistancePerRotation(0.0477); // Meters

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.leftPosition =
        Rotation2d.fromRotations(leftAbsoluteEncoder.getAbsolutePosition())
            .plus(leftPositionOffset);
    inputs.leftPositionMeters = leftAbsoluteEncoder.getDistance() + leftLinearPositionOffsetMeters;
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.leftAppliedCurrentAmps = new double[] {leftMotor.getOutputCurrent()};
    inputs.leftTemperatureCelsius = new double[] {leftMotor.getMotorTemperature()};

    inputs.rightPosition =
        Rotation2d.fromRotations(rightAbsoluteEncoder.getAbsolutePosition())
            .plus(rightPositionOffset);
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.rightAppliedCurrentAmps = new double[] {rightMotor.getOutputCurrent()};
    inputs.rightTemperatureCelsius = new double[] {rightMotor.getMotorTemperature()};
  }

  @Override
  public void setLeftVolts(double volts) {
    leftMotor.setVoltage(volts);
  }

  @Override
  public void setRightVolts(double volts) {
    rightMotor.setVoltage(volts);
  }
}
