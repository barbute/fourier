// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Targeting system is responsible for accepting data and calculating optimal states for subsystems
 * to shoot notes
 */
public class TargetingSystem {
  /** The Singleton Instance */
  private static TargetingSystem instance;

  private static InterpolatingDoubleTreeMap launchMap;

  /** Added to the angle returned by the launch map */
  private static Rotation2d shotCompensationDegrees = new Rotation2d();

  /** Filter to stabilize shaky distance estimates */
  private static LinearFilter distanceFilter = LinearFilter.movingAverage(10);

  private static Pose3d currentOdometryPose = new Pose3d();
  private static Pose3d currentFilteredPose = new Pose3d();

  private static boolean calculateWithVision = true;

  /** Returns the Targeting System's instance */
  public static TargetingSystem getInstance() {
    if (instance == null) {
      instance = new TargetingSystem();
    }
    return instance;
  }

  /**
   * Initializes the launch map, no fudge factors are added here. Launch map is in meters (key) and
   * degrees (value)
   */
  private static void initializeLaunchMap() {
    launchMap = new InterpolatingDoubleTreeMap();

    launchMap.put(Units.inchesToMeters(37.0), 55.0);
    launchMap.put(Units.inchesToMeters(47.0), 53.0);
    launchMap.put(Units.inchesToMeters(57.0), 52.0);
    launchMap.put(Units.inchesToMeters(67.0), 49.5);
    launchMap.put(Units.inchesToMeters(77.0), 47.0);
    launchMap.put(Units.inchesToMeters(87.0), 44.5);
    launchMap.put(Units.inchesToMeters(97.0), 42.0);
    launchMap.put(Units.inchesToMeters(107.0), 40.5);
    launchMap.put(Units.inchesToMeters(110.0), 40.3);
    launchMap.put(Units.inchesToMeters(120.0), 39.75);
    launchMap.put(Units.inchesToMeters(130.0), 38.5);
  }

  /**
   * Updates the current odometry pose that the Targeting System uses for calculations. Usually
   * called in Drive subsystem where the odometer is updated
   */
  public static void updateCurrentOdometryPosition(Pose3d robotPose) {
    currentOdometryPose = robotPose;
  }

  /**
   * Updates the current filtered pose that the Targeting System uses for calculations. Usually
   * called in Drive subsystem where the pose estimator and filter is updated
   */
  public static void updateCurrentFilteredPosition(Pose3d robotPose) {
    currentFilteredPose = robotPose;
  }

  /** Allows the operator to set whether or not to use vision with distance calculations */
  public static void setVisionUseage(boolean useVision) {
    calculateWithVision = useVision;
  }

  /**
   * Calculate distance between the current pose (held inside the Targeting System) and the target
   * pose passed into this function
   */
  public static DoubleSupplier calculateDistance(Pose3d targetPose) {
    if (calculateWithVision) {
      return () ->
          distanceFilter.calculate(
              targetPose.getTranslation().getDistance(currentFilteredPose.getTranslation()));
    } else {
      return () ->
          distanceFilter.calculate(
              targetPose.getTranslation().getDistance(currentOdometryPose.getTranslation()));
    }
  }

  /** Calculate the optimal heading for the robot to shoot at a target */
  public static Supplier<Rotation2d> calculateOptimalHeading(Pose3d targetPose) {
    if (calculateWithVision) {
      double deltaX = targetPose.getX() - currentFilteredPose.getX();
      double deltaY = targetPose.getY() - currentFilteredPose.getY();

      return () -> new Rotation2d(deltaX, deltaY);
    } else {
      double deltaX = targetPose.getX() - currentOdometryPose.getX();
      double deltaY = targetPose.getY() - currentOdometryPose.getY();

      return () -> new Rotation2d(deltaX, deltaY);
    }
  }

  /** Calculate the optimal angle for the shooter when passed in a taget pose */
  public static Supplier<Rotation2d> calculateOptimalAngle(Pose3d targetPose) {
    if (launchMap == null) {
      initializeLaunchMap();
    }

    double distanceM = calculateDistance(targetPose).getAsDouble();

    return () -> Rotation2d.fromDegrees(launchMap.get(distanceM)).plus(shotCompensationDegrees);
  }
}
