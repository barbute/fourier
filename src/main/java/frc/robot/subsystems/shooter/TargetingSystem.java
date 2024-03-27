// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * Targeting system is responsible for accepting data and calculating optimal states for subsystems
 * to shoot notes
 */
public class TargetingSystem {
  /** The Singleton Instance */
  private static TargetingSystem instance;

  private InterpolatingDoubleTreeMap launchMap;

  /** Added to the angle returned by the launch map */
  private Rotation2d shotCompensationDegrees = new Rotation2d();

  /** Filter to stabilize shaky distance estimates */
  private LinearFilter distanceFilter = LinearFilter.movingAverage(10);

  private Pose3d currentOdometryPose = new Pose3d();
  private Pose3d currentFilteredPose = new Pose3d();

  private boolean calculateWithVision = true;

  private double calculatedDistanceM = 0.0;
  private Rotation2d calculatedAngle = new Rotation2d();
  private Rotation2d calculatedHeading = new Rotation2d();

  /** Returns the Targeting System's instance */
  public static TargetingSystem getInstance() {
    if (instance == null) {
      instance = new TargetingSystem();
    }
    return instance;
  }

  /** Method that runs every loop cycle. Called in robotPeriodic() */
  public void periodic() {
    Logger.recordOutput("TargetingSystem/CalculatedDistance", calculatedDistanceM);
    Logger.recordOutput("TargetingSystem/CalculatedAngle", calculatedAngle);
    Logger.recordOutput("TargetingSystem/CalculatedHeading", calculatedHeading);
  }

  /**
   * Initializes the launch map, no fudge factors are added here. Launch map is in meters (key) and
   * degrees (value)
   */
  private void initializeLaunchMap() {
    launchMap = new InterpolatingDoubleTreeMap();

    launchMap.put(Units.inchesToMeters(2.0), 55.0);
    launchMap.put(Units.inchesToMeters(12.0), 53.0);
    launchMap.put(Units.inchesToMeters(22.0), 52.0);
    launchMap.put(Units.inchesToMeters(32.0), 49.5);
    launchMap.put(Units.inchesToMeters(42.0), 47.0);
    launchMap.put(Units.inchesToMeters(52.0), 44.5);
    launchMap.put(Units.inchesToMeters(62.0), 42.0);
    launchMap.put(Units.inchesToMeters(72.0), 40.5);
    launchMap.put(Units.inchesToMeters(75.0), 40.3);
    launchMap.put(Units.inchesToMeters(85.0), 39.75);
    launchMap.put(Units.inchesToMeters(95.0), 38.5);
  }

  /**
   * Updates the current odometry pose that the Targeting System uses for calculations. Usually
   * called in Drive subsystem where the odometer is updated
   */
  public void updateCurrentOdometryPosition(Pose3d robotPose) {
    currentOdometryPose = robotPose;
  }

  /**
   * Updates the current filtered pose that the Targeting System uses for calculations. Usually
   * called in Drive subsystem where the pose estimator and filter is updated
   */
  public void updateCurrentFilteredPosition(Pose3d robotPose) {
    currentFilteredPose = robotPose;
  }

  /** Allows the operator to set whether or not to use vision with distance calculations */
  public void setVisionUseage(boolean useVision) {
    calculateWithVision = useVision;
  }

  /**
   * Calculate distance between the current pose (held inside the Targeting System) and the target
   * pose passed into this function
   */
  public double calculateDistance(Pose3d targetPose) {
    if (calculateWithVision) {
      calculatedDistanceM =
          distanceFilter.calculate(
              targetPose.getTranslation().getDistance(currentFilteredPose.getTranslation()));
      return calculatedDistanceM;
    } else {
      calculatedDistanceM =
          distanceFilter.calculate(
              targetPose.getTranslation().getDistance(currentOdometryPose.getTranslation()));
      return calculatedDistanceM;
    }
  }

  /** Calculate the optimal heading for the robot to shoot at a target */
  public Rotation2d calculateOptimalHeading(Pose3d targetPose) {
    if (calculateWithVision) {
      double deltaX = targetPose.getX() - currentFilteredPose.getX();
      double deltaY = targetPose.getY() - currentFilteredPose.getY();

      calculatedHeading = new Rotation2d(deltaX, deltaY);

      return calculatedHeading;
    } else {
      double deltaX = targetPose.getX() - currentOdometryPose.getX();
      double deltaY = targetPose.getY() - currentOdometryPose.getY();

      calculatedHeading = new Rotation2d(deltaX, deltaY);

      return calculatedHeading;
    }
  }

  /** Calculate the optimal angle for the shooter when passed in a taget pose */
  public Rotation2d calculateOptimalAngle(Pose3d targetPose) {
    if (launchMap == null) {
      initializeLaunchMap();
    }

    double distanceM = calculateDistance(targetPose);
    calculatedAngle =
        Rotation2d.fromDegrees(launchMap.get(distanceM)).plus(shotCompensationDegrees);

    return calculatedAngle;
  }
}
