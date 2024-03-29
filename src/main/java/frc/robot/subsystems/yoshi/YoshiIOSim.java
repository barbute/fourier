// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Physics sim implementation of YoshiIO */
public class YoshiIOSim implements YoshiIO {
  private final double LOOP_PERIOD_SECS = 0.02;
  private final double PIVOT_GEAR_RATIO = (64.0 / 1.0) * (3.0 / 1.0);
  private final double FLYWHEEL_GEAR_RATIO = 5.0 / 1.0;

  private SingleJointedArmSim pivotMotor =
      new SingleJointedArmSim(
          DCMotor.getNeo550(1),
          PIVOT_GEAR_RATIO,
          0.002,
          0.5,
          Math.toRadians(-49.0),
          Math.toRadians(120.0),
          true,
          Math.toRadians(90.0));
  private FlywheelSim flywheelMotor =
      new FlywheelSim(DCMotor.getNeo550(1), FLYWHEEL_GEAR_RATIO, 0.002);

  private double pivotAppliedVolts = 0.0;
  private double flywheelAppliedVolts = 0.0;

  @Override
  public void updateInputs(YoshiIOInputs inputs) {
    pivotMotor.update(LOOP_PERIOD_SECS);
    flywheelMotor.update(LOOP_PERIOD_SECS);

    inputs.pivotPosition = Rotation2d.fromRadians(pivotMotor.getAngleRads());
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotAppliedCurrentAmps = new double[] {pivotMotor.getCurrentDrawAmps()};
    inputs.pivotTemperatureCelsius = new double[] {0.0};

    inputs.flywheelAppliedVolts = flywheelAppliedVolts;
    inputs.flywheelAppliedCurrentAmps = new double[] {flywheelMotor.getCurrentDrawAmps()};
    inputs.flywheelTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setPivotVolts(double volts) {
    pivotAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotMotor.setInputVoltage(pivotAppliedVolts);
  }

  @Override
  public void setFlywheelVolts(double volts) {
    flywheelAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    flywheelMotor.setInputVoltage(flywheelAppliedVolts);
  }
}
