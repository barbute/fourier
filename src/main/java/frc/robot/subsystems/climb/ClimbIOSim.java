// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Physics sim implementation of ClimbIO */
public class ClimbIOSim implements ClimbIO {
  private final double LOOP_PERIOD_SECS = 0.02;
  private final double GEAR_RATIO = 125.0 / 1.0;

  private SingleJointedArmSim leftMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          GEAR_RATIO,
          0.005,
          Units.inchesToMeters(12.5),
          -Math.PI,
          Math.PI,
          true,
          Math.toRadians(180.0));
  private SingleJointedArmSim rightMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          GEAR_RATIO,
          0.005,
          Units.inchesToMeters(12.5),
          -Math.PI,
          Math.PI,
          true,
          Math.toRadians(180.0));

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    leftMotor.update(LOOP_PERIOD_SECS);
    rightMotor.update(LOOP_PERIOD_SECS);

    inputs.leftPosition = Rotation2d.fromRadians(leftMotor.getAngleRads());
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftAppliedCurrentAmps = new double[] {leftMotor.getCurrentDrawAmps()};
    inputs.leftTemperatureCelsius = new double[] {0.0};

    inputs.rightPosition = Rotation2d.fromRadians(rightMotor.getAngleRads());
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightAppliedCurrentAmps = new double[] {rightMotor.getCurrentDrawAmps()};
    inputs.rightTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setLeftVolts(double volts) {
    leftAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    leftMotor.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void setRightVolts(double volts) {
    rightAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    rightMotor.setInputVoltage(rightAppliedVolts);
  }
}
