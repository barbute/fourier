// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

/** Physics sim implementation of LauncherIO */
public class LauncherIOSim implements LauncherIO {
  @Override
  public void updateInputs(LauncherIOInputs inputs) {}

  @Override
  public void setTopVolts(double volts) {}

  @Override
  public void setBottomVolts(double volts) {}

  @Override
  public void setTopVelocityMPS(double velocitySetpointMPS, double accelerationSetpointMPS) {}

  @Override
  public void setBottomVelocityMPS(double velocitySetpointMPS, double accelerationSetpointMPS) {}
}
