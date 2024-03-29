// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  // Relative to facing the same direction as the shooter barrel
  private VisionIO frontLeftIO;
  private VisionIO frontRightIO;

  private VisionIOInputsAutoLogged frontLeftIOInputs = new VisionIOInputsAutoLogged();
  private VisionIOInputsAutoLogged frontRightIOInputs = new VisionIOInputsAutoLogged();

  private LoggedTunableNumber singleTagStdDevX;
  private LoggedTunableNumber singleTagStdDevY;
  private LoggedTunableNumber singleTagStdDevTheta;

  private LoggedTunableNumber multiTagStdDevX;
  private LoggedTunableNumber multiTagStdDevY;
  private LoggedTunableNumber multiTagStdDevTheta;

  public Vision(VisionIO frontLeftIO, VisionIO frontRightIO) {
    this.frontLeftIO = frontLeftIO;
    this.frontRightIO = frontRightIO;

    singleTagStdDevX =
        new LoggedTunableNumber(
            "Vision/SingleTagDevsX", this.frontLeftIO.getSingleTagStdDevsCoeff().get(0, 0));
    singleTagStdDevY =
        new LoggedTunableNumber(
            "Vision/SingleTagDevsY", this.frontLeftIO.getSingleTagStdDevsCoeff().get(1, 0));
    singleTagStdDevTheta =
        new LoggedTunableNumber(
            "Vision/SingleTagDevsTheta", this.frontLeftIO.getSingleTagStdDevsCoeff().get(2, 0));

    multiTagStdDevX =
        new LoggedTunableNumber(
            "Vision/MultiTagDevsX", this.frontLeftIO.getMultiTagStdDevsCoeff().get(0, 0));
    multiTagStdDevY =
        new LoggedTunableNumber(
            "Vision/MultiTagDevsY", this.frontLeftIO.getMultiTagStdDevsCoeff().get(1, 0));
    multiTagStdDevTheta =
        new LoggedTunableNumber(
            "Vision/MultiTagDevsTheta", this.frontLeftIO.getMultiTagStdDevsCoeff().get(2, 0));
  }

  @Override
  public void periodic() {
    frontLeftIO.updateInputs(frontLeftIOInputs);
    Logger.processInputs("Vision/FrontLeft", frontLeftIOInputs);
    frontRightIO.updateInputs(frontRightIOInputs);
    Logger.processInputs("Vision/FrontRight", frontRightIOInputs);

    if (Constants.debuggingMode) {
      updateTunableNumbers();
    }
  }

  /** Method to update tunable numbers */
  private void updateTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          frontLeftIO.setSingleTagStdDevs(
              singleTagStdDevX.get(), singleTagStdDevY.get(), singleTagStdDevTheta.get());
          frontRightIO.setSingleTagStdDevs(
              singleTagStdDevX.get(), singleTagStdDevY.get(), singleTagStdDevTheta.get());
        },
        singleTagStdDevX,
        singleTagStdDevY,
        singleTagStdDevTheta);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          frontLeftIO.setMultiTagStdDevs(
              multiTagStdDevX.get(), multiTagStdDevY.get(), multiTagStdDevTheta.get());
          frontRightIO.setMultiTagStdDevs(
              multiTagStdDevX.get(), multiTagStdDevY.get(), multiTagStdDevTheta.get());
        },
        multiTagStdDevX,
        multiTagStdDevY,
        multiTagStdDevTheta);
  }
}
