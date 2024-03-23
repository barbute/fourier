// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    public double topVelocityMPS = 0.0;
    public double topVelocitySetpointMPS = 0.0;
    public double topVelocityErrorMPS = 0.0;
    public double topAppliedVolts = 0.0;
    public double[] topAppliedCurrentAmps = new double[] {0.0};
    public double[] topTemperatureCelsius = new double[] {0.0};

    public double bottomVelocityMPS = 0.0;
    public double bottomVelocitySetpointMPS = 0.0;
    public double bottomVelocityErrorMPS = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double[] bottomAppliedCurrentAmps = new double[] {0.0};
    public double[] bottomTemperatureCelsius = new double[] {0.0};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LauncherIOInputs inputs) {}

  /** Run the top flywheel motor at the specified voltage. */
  public default void setTopVolts(double volts) {}
  
  /** Run the bottom flywheel motor at the specified voltage. */
  public default void setBottomVolts(double volts) {}

  /** Run the top motor velocity setpoint for closed-loop control */
  public default void setTopVelocityMPS(double velocitySetpointMPS) {}

  /** Run the bottom motor velocity setpoint for closed-loop control */
  public default void setBottomVelocityMPS(double velocitySetpointMPS) {}
}
