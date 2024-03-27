// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.debugging;

import edu.wpi.first.math.geometry.Rotation2d;

/** Class to hold fudge factors to update based off of the field */
public class FudgeFactors {
  /** Blue alliance fudge factors */
  public static class Blue {
    /**
     * Compensation for the shot map angle, value is added to the returned shot map value in
     * Targeting System
     */
    public static final Rotation2d SHOT_POSITION_COMPENSATION = Rotation2d.fromDegrees(0.0);

    /**
     * Compensation for the calculated distance, value is added to the returned value in
     * calculateDistance() in Targeting System
     */
    public static final double SHOT_DISTANCE_COMPENSATION_M = 0.0;
  }

  /** Red alliance fudge factors */
  public static class Red {
    /**
     * Compensation for the shot map, value is added to the returned shot map value in Targeting
     * System
     */
    public static final Rotation2d SHOT_POSITION_COMPENSATION = Rotation2d.fromDegrees(0.0);

    /**
     * Compensation for the calculated distance, value is added to the returned value in
     * calculateDistance() in Targeting System
     */
    public static final double SHOT_DISTANCE_COMPENSATION_M = 0.0;
  }
}
