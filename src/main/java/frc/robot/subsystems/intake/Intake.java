// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public enum IntakeSetpoints {
    INTAKE(12.0),
    OUTTAKE(-12.0),
    SPEAKER_SHOOT(12.0),
    AMP_SHOOT(12.0),
    STOPPED(0.0);

    private double setpointVolts;

    IntakeSetpoints(double volts) {
      this.setpointVolts = volts;
    }

    public double getVolts() {
      return this.setpointVolts;
    }
  }

  private IntakeSetpoints currentSetpoint = IntakeSetpoints.STOPPED;

  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged intakeIOInputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    intakeIO = io;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeIOInputs);
    Logger.processInputs("Intake", intakeIOInputs);

    if (DriverStation.isDisabled()) {
      currentSetpoint = IntakeSetpoints.STOPPED;
      intakeIO.setVolts(0.0);
    }

    if (currentSetpoint != null) {
      intakeIO.setVolts(currentSetpoint.getVolts());
    }
  }

  /** Run the intake at a setpoint voltage */
  public void runIntake(IntakeSetpoints setpoint) {
    currentSetpoint = setpoint;
  }

  /** Returns the current setpoint state of the Intake */
  @AutoLogOutput(key = "Intake/CurrentSetpoint")
  public IntakeSetpoints getCurrentSetpoint() {
    if (currentSetpoint != null) {
      return currentSetpoint;
    } else {
      return IntakeSetpoints.STOPPED;
    }
  }
}
