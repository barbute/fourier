// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

/** IndexerIO implementation for SparkMax motor controller (NEO) */
public class IndexerIOSparkMax implements IndexerIO {
  private final double GEAR_RATIO = 5.0 / 1.0;

  private CANSparkMax indexerMotor = new CANSparkMax(42, MotorType.kBrushless);
  private RelativeEncoder indexerEncoder = indexerMotor.getEncoder();

  private DigitalInput indexerSensor = new DigitalInput(7);

  public IndexerIOSparkMax() {
    indexerMotor.clearFaults();
    indexerMotor.restoreFactoryDefaults();

    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);
    indexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);

    indexerMotor.setSmartCurrentLimit(30);
    indexerMotor.enableVoltageCompensation(12.0);
    indexerMotor.setIdleMode(IdleMode.kBrake);

    indexerMotor.setInverted(false);

    indexerMotor.burnFlash();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocityRPM = indexerEncoder.getVelocity() / GEAR_RATIO;
    inputs.appliedVolts = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
    inputs.appliedCurrentAmps = new double[] {indexerMotor.getOutputCurrent()};
    inputs.temperatureCelsius = new double[] {indexerMotor.getMotorTemperature()};

    // Negated cause that's how sensor works ig
    inputs.beamBroken = !indexerSensor.get();
  }

  @Override
  public void setVolts(double volts) {
    indexerMotor.setVoltage(volts);
  }
}
