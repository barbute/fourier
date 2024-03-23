// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class AnglerIOSim implements AnglerIO {
  private final double LOOP_PERIOD_SECS = 0.02;
  private final double GEAR_RATIO = 1.0 / 1.0;

  private SingleJointedArmSim anglerMotor = new SingleJointedArmSim(DCMotor.getNEO(1), GEAR_RATIO, 0.167248163371, 0.4826, Math.toRadians(20.0), Math.toRadians(60.0), false, Math.toRadians(45.0), VecBuilder.fill(0.0));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(AnglerIOInputs inputs) {
    anglerMotor.update(LOOP_PERIOD_SECS);

    inputs.absolutePosition = Rotation2d.fromRadians(anglerMotor.getAngleRads());
    inputs.relativePosition = Rotation2d.fromRadians(anglerMotor.getAngleRads());
    inputs.anglerDutyCycleFrequency = 955;
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {anglerMotor.getCurrentDrawAmps()};
    inputs.temperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    anglerMotor.setInputVoltage(appliedVolts);
  }
}
