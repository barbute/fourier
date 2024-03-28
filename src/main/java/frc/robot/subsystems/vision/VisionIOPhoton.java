// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** VisionIO implementation for using Photon Vision on a real camera */
public class VisionIOPhoton implements VisionIO {

  public VisionIOPhoton() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {}

  @Override
  public void setSingleTagStdDevs(double x, double y, double theta) {}

  @Override
  public void setMultiTagStdDevs(double x, double y, double theta) {}

  @Override
  public Matrix<N3, N1> getSingleTagStdDevsCoeff() {
    return VecBuilder.fill(0.0, 0.0, 0.0);
  }

  @Override
  public Matrix<N3, N1> getMultiTagStdDevsCoeff() {
    return VecBuilder.fill(0.0, 0.0, 0.0);
  }
}
