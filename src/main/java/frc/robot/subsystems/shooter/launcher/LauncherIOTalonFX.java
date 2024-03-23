// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

/** LauncherIO implementation for TalonFX motor controller (Falcon500) */
public class LauncherIOTalonFX implements LauncherIO {
  private final double GEARING = 1.0 / 1.0;
  private final double RADIUS_M = 6.35 / 100;
  private final double CIRCUMFRENCE_M = 2.0 * Math.PI * RADIUS_M;

  public LauncherIOTalonFX() {}

  @Override
  public void updateInputs(LauncherIOInputs inputs) {}

  @Override
  public void setTopVolts(double volts) {}

  @Override
  public void setBottomVolts(double volts) {}

  @Override
  public void setTopVelocityMPS(double velocitySetpointMPS) {}

  @Override
  public void setBottomVelocityMPS(double velocitySetpointMPS) {}
}
