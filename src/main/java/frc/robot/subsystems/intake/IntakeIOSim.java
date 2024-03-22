// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Physics sim implementation of IntakeIO */
public class IntakeIOSim implements IntakeIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final double GEAR_RATIO = 9.0 / 1.0;

  private FlywheelSim intakeMotor = new FlywheelSim(DCMotor.getNEO(1), GEAR_RATIO, 0.0002);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotor.update(LOOP_PERIOD_SECS);

    inputs.velocityRPM = intakeMotor.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {Math.abs(intakeMotor.getCurrentDrawAmps())};
    inputs.temperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    intakeMotor.setInputVoltage(appliedVolts);
  }
}
