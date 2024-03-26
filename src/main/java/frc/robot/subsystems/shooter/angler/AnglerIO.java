// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.angler;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AnglerIO {
  @AutoLog
  public static class AnglerIOInputs {
    public Rotation2d absolutePosition = new Rotation2d();
    public Rotation2d relativePosition = new Rotation2d();
    public int anglerDutyCycleFrequency = 0;
    public double appliedVolts = 0.0;
    public double[] appliedCurrentAmps = new double[] {0.0};
    public double[] temperatureCelsius = new double[] {0.0};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AnglerIOInputs inputs) {}

  /** Run the angler motor at the specified voltage. */
  public default void setVolts(double volts) {}
}
