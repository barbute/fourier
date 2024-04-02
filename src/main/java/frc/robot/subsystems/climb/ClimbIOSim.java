// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

/** Physics sim implementation of ClimbIO */
public class ClimbIOSim implements ClimbIO {
  private final double GEAR_RATIO = 125.0 / 1.0;

  @Override
  public void updateInputs(ClimbIOInputs inputs) {}

  @Override
  public void setLeftVolts(double volts) {}

  @Override
  public void setRightVolts(double volts) {}
}
