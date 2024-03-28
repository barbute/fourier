// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshi;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface YoshiIO {
  @AutoLog
  public static class YoshiIOInputs {
    public Rotation2d pivotPosition = new Rotation2d();
    public double pivotAppliedVolts = 0.0;
    public double[] pivotAppliedCurrentAmps = new double[] {0.0};
    public double[] pivotTemperatureCelsius = new double[] {0.0};

    public double flywheelAppliedVolts = 0.0;
    public double[] flywheelAppliedCurrentAmps = new double[] {0.0};
    public double[] flywheelTemperatureCelsius = new double[] {0.0};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(YoshiIOInputs inputs) {}

  /** Run the pivot motor at the specified voltage. */
  public default void setPivotVolts(double volts) {}

  /** Run the flywheel motor at the specified voltage. */
  public default void setFlywheelVolts(double volts) {}
}
