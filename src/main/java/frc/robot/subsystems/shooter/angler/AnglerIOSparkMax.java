// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

/** AnglerIO implementation for SparkMax motor controller (NEO) */
public class AnglerIOSparkMax implements AnglerIO {
  private CANSparkMax anglerMotor = new CANSparkMax(41, MotorType.kBrushless);

  private DutyCycleEncoder absoluteAnglerEncoder = new DutyCycleEncoder(0);
  private Encoder relativeAnglerEncoder = new Encoder(5, 6);

  private Rotation2d absoluteEncoderOffset = Rotation2d.fromDegrees(179.0);

  public AnglerIOSparkMax() {
    anglerMotor.clearFaults();
    anglerMotor.restoreFactoryDefaults();

    anglerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    anglerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    anglerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    anglerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    anglerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    anglerMotor.setSmartCurrentLimit(60);
    anglerMotor.enableVoltageCompensation(12.0);
    anglerMotor.setIdleMode(IdleMode.kBrake);

    anglerMotor.setInverted(true);

    anglerMotor.burnFlash();

    absoluteAnglerEncoder.setDutyCycleRange(1.0 / 8192.0, 8191.0 / 8192.0);
  }

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    inputs.absolutePosition =
        Rotation2d.fromDegrees(360.0)
            .minus(
                Rotation2d.fromRotations(absoluteAnglerEncoder.getAbsolutePosition())
                    .plus(absoluteEncoderOffset));
    inputs.relativePosition = Rotation2d.fromRotations(relativeAnglerEncoder.get() / 2048.0);
    inputs.anglerDutyCycleFrequency = absoluteAnglerEncoder.getFrequency();
    inputs.appliedVolts = anglerMotor.getAppliedOutput() * anglerMotor.getBusVoltage();
    inputs.appliedCurrentAmps = new double[] {anglerMotor.getOutputCurrent()};
    inputs.temperatureCelsius = new double[] {anglerMotor.getMotorTemperature()};
  }

  @Override
  public void setVolts(double volts) {
    anglerMotor.setVoltage(volts);
  }
}
