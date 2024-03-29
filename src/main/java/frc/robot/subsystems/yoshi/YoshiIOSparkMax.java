// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshi;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** YoshiIO implementation for two SparkMax motor controllers (NEO 550s) */
public class YoshiIOSparkMax implements YoshiIO {
  private CANSparkMax pivotMotor = new CANSparkMax(52, MotorType.kBrushless);
  private CANSparkMax flywheelMotor = new CANSparkMax(53, MotorType.kBrushless);

  private DutyCycleEncoder pivotAbsoluteEncoder = new DutyCycleEncoder(8);

  private Rotation2d pivotAbsoluteOffset = Rotation2d.fromDegrees(73.0);

  public YoshiIOSparkMax() {
    pivotMotor.clearFaults();
    pivotMotor.restoreFactoryDefaults();

    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    pivotMotor.setSmartCurrentLimit(40);
    pivotMotor.enableVoltageCompensation(12.0);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    pivotMotor.setInverted(true);

    pivotMotor.burnFlash();

    pivotAbsoluteEncoder.setDutyCycleRange(1.0 / 8192.0, 8191.0 / 8192.0);

    flywheelMotor.clearFaults();
    flywheelMotor.restoreFactoryDefaults();

    flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 50);
    flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    flywheelMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    flywheelMotor.setSmartCurrentLimit(40);
    flywheelMotor.enableVoltageCompensation(12.0);
    flywheelMotor.setIdleMode(IdleMode.kBrake);

    flywheelMotor.setInverted(false);

    flywheelMotor.burnFlash();
  }

  @Override
  public void updateInputs(YoshiIOInputs inputs) {
    inputs.pivotPosition =
        Rotation2d.fromRotations(pivotAbsoluteEncoder.getAbsolutePosition())
            .minus(pivotAbsoluteOffset)
            .times(-1.0);
    inputs.pivotAppliedVolts = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.pivotAppliedCurrentAmps = new double[] {pivotMotor.getOutputCurrent()};
    inputs.pivotTemperatureCelsius = new double[] {pivotMotor.getMotorTemperature()};

    inputs.flywheelAppliedVolts = flywheelMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.flywheelAppliedCurrentAmps = new double[] {flywheelMotor.getOutputCurrent()};
    inputs.flywheelTemperatureCelsius = new double[] {flywheelMotor.getMotorTemperature()};
  }

  @Override
  public void setPivotVolts(double volts) {
    pivotMotor.setVoltage(volts);
  }

  @Override
  public void setFlywheelVolts(double volts) {
    flywheelMotor.setVoltage(volts);
  }
}
