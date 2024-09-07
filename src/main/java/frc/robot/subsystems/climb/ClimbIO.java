// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public Rotation2d leftPosition = new Rotation2d();
    public double leftPositionMeters = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftAppliedCurrentAmps = new double[] {0.0};
    public double[] leftTemperatureCelsius = new double[] {0.0};

    public Rotation2d rightPosition = new Rotation2d();
    public double rightPositionMeters = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightAppliedCurrentAmps = new double[] {0.0};
    public double[] rightTemperatureCelsius = new double[] {0.0};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /** Run the left motor at the specified voltage. */
  public default void setLeftVolts(double volts) {}

  /** Run the right motor at the specified voltage. */
  public default void setRightVolts(double volts) {}
}
