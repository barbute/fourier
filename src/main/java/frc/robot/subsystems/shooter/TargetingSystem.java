// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.debugging.Alert;
import frc.robot.util.debugging.Alert.AlertType;
import frc.robot.util.debugging.FudgeFactors;
import org.littletonrobotics.junction.Logger;

/**
 * Targeting system is responsible for accepting data and calculating optimal states for subsystems
 * to shoot notes
 */
public class TargetingSystem {
  /** The Singleton Instance */
  private static TargetingSystem instance;

  private InterpolatingDoubleTreeMap launchMap;

  /** Filter to stabilize shaky distance estimates */
  private LinearFilter distanceFilter = LinearFilter.movingAverage(10);

  private Pose3d currentOdometryPose = new Pose3d();
  private Pose3d currentFilteredPose = new Pose3d();

  private boolean calculateWithVision = false;
  private boolean useFudgeFactors = false;

  // Debugging data
  private double calculatedDistanceM = 0.0;
  private Rotation2d calculatedAngle = new Rotation2d();
  private Rotation2d calculatedHeading = new Rotation2d();
  private Pose3d targetPoseAngle = new Pose3d();

  private Alert useVisionAlert = new Alert("Console", "CALCULATING WITH VISION", AlertType.INFO);
  private Alert useFudgeFactorsAlert =
      new Alert("Console", "USING FUDGE FACTORS", AlertType.WARNING);

  /** Returns the Targeting System's instance */
  public static TargetingSystem getInstance() {
    if (instance == null) {
      instance = new TargetingSystem();
    }
    return instance;
  }

  /** Method that runs every loop cycle. Called in robotPeriodic() */
  public void periodic() {
    useVisionAlert.set(calculateWithVision);
    useFudgeFactorsAlert.set(useFudgeFactors);

    Logger.recordOutput("TargetingSystem/CalculatedDistance", calculatedDistanceM);
    Logger.recordOutput("TargetingSystem/CalculatedAngle", calculatedAngle);
    Logger.recordOutput("TargetingSystem/CalculatedHeading", calculatedHeading);
    Logger.recordOutput("TargetingSystem/TargetPoseAngler", targetPoseAngle);
  }

  /**
   * Initializes the launch map, no fudge factors are added here. Launch map is in meters (key) and
   * degrees (value)
   */
  private void initializeLaunchMap() {
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

  /** Returns the desired value with fudge factors (if applicable) */
  public double applyFudgeFactors(double x) {
    if (useFudgeFactors) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red) {
        return x + FudgeFactors.Red.SHOT_DISTANCE_COMPENSATION_M;
      }
      return x + FudgeFactors.Blue.SHOT_DISTANCE_COMPENSATION_M;
    }
    return x;
  }

  /** Returns the desired value with fudge factors (if applicable) */
  public Rotation2d applyFudgeFactors(Rotation2d x) {
    if (useFudgeFactors) {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red) {
        return x.plus(FudgeFactors.Red.SHOT_POSITION_COMPENSATION);
      }
      return x.plus(FudgeFactors.Blue.SHOT_POSITION_COMPENSATION);
    }
    return x;
  }

  /**
   * Calculate distance between the current pose (held inside the Targeting System) and the target
   * pose passed into this function
   */
  public double calculateDistance(Pose3d targetPose) {
    Pose3d currentPose = (calculateWithVision) ? currentFilteredPose : currentOdometryPose;

    Translation2d currentPose2d = new Translation2d(currentPose.getX(), currentPose.getY());
    Translation2d targetPose2d = new Translation2d(targetPose.getX(), targetPose.getY());

    calculatedDistanceM =
        applyFudgeFactors(distanceFilter.calculate(currentPose2d.getDistance(targetPose2d)));
    return calculatedDistanceM;
  }

  /** Calculate the optimal heading for the robot to shoot at a target */
  public Rotation2d calculateOptimalHeading(Pose3d targetPose) {
    // TODO Consider switch to pose2d and just use pose3d.toPose2d()
    Pose3d currentPose = (calculateWithVision) ? currentFilteredPose : currentOdometryPose;

    double deltaX = targetPose.getX() - currentPose.getX();
    double deltaY = targetPose.getY() - currentPose.getY();

    calculatedHeading = applyFudgeFactors(new Rotation2d(deltaX, deltaY));

    return calculatedHeading;
  }

  /** Calculate the optimal angle for the shooter when passed in a taget pose */
  public Rotation2d calculateOptimalAngle(Pose3d targetPose) {
    if (launchMap == null) {
      initializeLaunchMap();
    }
    targetPoseAngle = targetPose;

    double distanceM = calculateDistance(targetPose);
    calculatedAngle = applyFudgeFactors(Rotation2d.fromDegrees(launchMap.get(distanceM)));

    return calculatedAngle;
  }
}
