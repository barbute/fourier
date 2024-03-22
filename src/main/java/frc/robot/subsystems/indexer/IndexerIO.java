// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double[] appliedCurrentAmps = new double[] {0.0};
    public double[] temperatureCelsius = new double[] {0.0};

    public boolean beamBroken = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IndexerIOInputs inputs) {}

  /** Run the indexer motor at the specified voltage. */
  public default void setVolts(double volts) {}
}
