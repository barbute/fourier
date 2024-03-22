// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.debugging.LoggedTunableNumber;

public class Indexer extends SubsystemBase {
  public enum IndexerSetpoints {
    INTAKE(() -> 12.0),
    STOW(() -> 4.0),
    OUTTAKE(() -> -12.0),
    SPEAKER_SHOOT(() -> 12.0),
    AMP_SHOOT(() -> 7.0),
    CUSTOM(new LoggedTunableNumber("Indexer/VoltageSetpoint", 4.0)),
    STOPPED(() -> 0.0);

    private DoubleSupplier setpointVolts;

    IndexerSetpoints(DoubleSupplier volts) {
      this.setpointVolts = volts;
    }

    public double getVolts() {
      return this.setpointVolts.getAsDouble();
    }
  }

  private IndexerSetpoints currentSetpoint = IndexerSetpoints.STOPPED;

  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged indexerIOInputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    indexerIO = io;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerIOInputs);
    Logger.processInputs("Indexer", indexerIOInputs);

    if (DriverStation.isDisabled()) {
      currentSetpoint = IndexerSetpoints.STOPPED;
      indexerIO.setVolts(0.0);
    }

    if (currentSetpoint != null) {
      indexerIO.setVolts(currentSetpoint.getVolts());
    }
  }

  /** Run the indexer at a setpoint voltage */
  public void runIndexer(IndexerSetpoints setpoint) {
    currentSetpoint = setpoint;
  }

  /** Returns the current setpoint state of the Indexer */
  @AutoLogOutput(key = "Indexer/CurrentSetpoint")
  public IndexerSetpoints getCurrentSetpoint() {
    if (currentSetpoint != null) {
      return currentSetpoint;
    }
    else {
      return IndexerSetpoints.STOPPED;
    }
  }
}
