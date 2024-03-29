// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean isConnected = false;
    public boolean hasTarget = false;
    public boolean hasTargetDebounced = false;
    public boolean hasSpeakerTarget = false;

    public double latestTimestampS = 0.0;
    public double latencyS = 0.0;
    public int numberOfTargets = 0;
    public int aprilTagID = 0;

    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;

    public Transform3d cameraToAprilTag = new Transform3d();
    public Transform3d robotToAprilTag = new Transform3d();
    public Transform2d speakerTagTransform = new Transform2d();
    public Pose2d speakerTagPose = new Pose2d();
    public Pose2d estimatedRobotPose = new Pose2d();

    public double poseAmbiguity = 0.0;

    public double xStdDev = 0.0;
    public double yStdDev = 0.0;
    public double thetaStdDev = 0.0;

    public double speakerXStdDev = 0.0;
    public double speakerYStdDev = 0.0;
    public double speakerThetaDev = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Sets the standard deviation for single-tag detection */
  public default void setSingleTagStdDevs(double x, double y, double theta) {}

  /** Sets the standard deviation for multi-tag detection */
  public default void setMultiTagStdDevs(double x, double y, double theta) {}

  /** Returns the standard deviation for single-tag detection */
  public default Matrix<N3, N1> getSingleTagStdDevsCoeff() {
    return VecBuilder.fill(0.0, 0.0, 0.0);
  }

  /** Returns the standard deviation for multi-tag detection */
  public default Matrix<N3, N1> getMultiTagStdDevsCoeff() {
    return VecBuilder.fill(0.0, 0.0, 0.0);
  }
}
