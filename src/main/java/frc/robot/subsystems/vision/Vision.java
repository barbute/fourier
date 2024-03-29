// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.debugging.LoggedTunableNumber;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  /** Cameras relative to the direction of the shooter barrel */
  public enum Camera {
    /**
     * The front-left limelight camera relative to facing the same direction as the intake-side of
     * the robot
     */
    FRONT_LEFT_0,
    /**
     * The front-right limelight camera relative to facing the same direction as the intake-side of
     * the robot
     */
    FRONT_RIGHT_1
  }

  private ArrayList<VisionIO> visionIOs = new ArrayList<VisionIO>();
  private ArrayList<VisionIOInputsAutoLogged> visionIOInputs =
      new ArrayList<VisionIOInputsAutoLogged>();

  private LoggedTunableNumber singleTagStdDevX;
  private LoggedTunableNumber singleTagStdDevY;
  private LoggedTunableNumber singleTagStdDevTheta;

  private LoggedTunableNumber multiTagStdDevX;
  private LoggedTunableNumber multiTagStdDevY;
  private LoggedTunableNumber multiTagStdDevTheta;

  public Vision(ArrayList<VisionIO> ios) {
    visionIOs = ios;
    for (int i = 0; i < visionIOs.size(); i++) {
      visionIOInputs.add(new VisionIOInputsAutoLogged());
    }

    singleTagStdDevX =
        new LoggedTunableNumber(
            "Vision/SingleTagDevsX", visionIOs.get(0).getSingleTagStdDevsCoeff().get(0, 0));
    singleTagStdDevY =
        new LoggedTunableNumber(
            "Vision/SingleTagDevsY", visionIOs.get(0).getSingleTagStdDevsCoeff().get(1, 0));
    singleTagStdDevTheta =
        new LoggedTunableNumber(
            "Vision/SingleTagDevsTheta", visionIOs.get(0).getSingleTagStdDevsCoeff().get(2, 0));

    multiTagStdDevX =
        new LoggedTunableNumber(
            "Vision/MultiTagDevsX", visionIOs.get(0).getMultiTagStdDevsCoeff().get(0, 0));
    multiTagStdDevY =
        new LoggedTunableNumber(
            "Vision/MultiTagDevsY", visionIOs.get(0).getMultiTagStdDevsCoeff().get(1, 0));
    multiTagStdDevTheta =
        new LoggedTunableNumber(
            "Vision/MultiTagDevsTheta", visionIOs.get(0).getMultiTagStdDevsCoeff().get(2, 0));
  }

  @Override
  public void periodic() {
    for (int i = 0; i < visionIOs.size(); i++) {
      visionIOs.get(i).updateInputs(visionIOInputs.get(i));
      String logKey = "";

      switch (i) {
        case 0:
          logKey = "Vision/FrontLeft";
          break;
        case 1:
          logKey = "Vision/FrontRight";
          break;
        default:
          logKey = "Vision/" + i;
          break;
      }

      Logger.processInputs(logKey, visionIOInputs.get(i));
    }

    if (Constants.debuggingMode) {
      updateTunableNumbers();
    }
  }

  /** Method to update tunable numbers */
  private void updateTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          for (VisionIO io : visionIOs) {
            io.setSingleTagStdDevs(
                singleTagStdDevX.get(), singleTagStdDevY.get(), singleTagStdDevTheta.get());
          }
        },
        singleTagStdDevX,
        singleTagStdDevY,
        singleTagStdDevTheta);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          for (VisionIO io : visionIOs) {
            io.setMultiTagStdDevs(
                multiTagStdDevX.get(), multiTagStdDevY.get(), multiTagStdDevTheta.get());
          }
        },
        multiTagStdDevX,
        multiTagStdDevY,
        multiTagStdDevTheta);
  }

  /** Returns whether or not the camera has an april tag */
  public boolean hasTarget(int index) {
    return visionIOInputs.get(index).hasTarget;
  }

  /** Returns the estimated pose from the camera */
  public Pose2d getEstimatedRobotPose(int index) {
    return visionIOInputs.get(index).estimatedRobotPose;
  }

  /** Returns the latest timestamp in seconds */
  public double getLatestTimestampS(int index) {
    return visionIOInputs.get(index).latestTimestampS;
  }

  /** Returns the x standard deviation value */
  public double getXStandardDeviation(int index) {
    return visionIOInputs.get(index).xStdDev;
  }

  /** Returns the y standard deviation value */
  public double getYStandardDeviation(int index) {
    return visionIOInputs.get(index).yStdDev;
  }

  /** Returns the theta standard deviation value */
  public double getThetaStandardDeviation(int index) {
    return visionIOInputs.get(index).thetaStdDev;
  }

  /** Returns the number of IO implementations in use */
  public int getNumberOfIOs() {
    return visionIOs.size();
  }
}
