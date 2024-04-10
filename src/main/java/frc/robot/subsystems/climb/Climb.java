// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbVisualizer.ClimbSide;
import frc.robot.util.debugging.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  public enum ClimbSetpoints {
    BOTH_IN(() -> 12.0, () -> 12.0),
    BOTH_OUT(() -> -12.0, () -> -12.0),
    CUSTOM(
        () -> new LoggedTunableNumber("Climb/LeftVoltageSetpoint", 4.0).get(),
        () -> new LoggedTunableNumber("Climb/RightVoltageSetpoint", 4.0).get()),
    STOPPED(() -> 0.0, () -> 0.0);

    private DoubleSupplier leftSetpointVolts;
    private DoubleSupplier rightSetpointVolts;

    ClimbSetpoints(DoubleSupplier leftVolts, DoubleSupplier rightVolts) {
      this.leftSetpointVolts = leftVolts;
      this.rightSetpointVolts = rightVolts;
    }

    public double getLeftVolts() {
      return this.leftSetpointVolts.getAsDouble();
    }

    public double getRightVolts() {
      return this.rightSetpointVolts.getAsDouble();
    }
  }

  private ClimbSetpoints currentSetpoint = ClimbSetpoints.STOPPED;

  private ClimbIO climbIO;
  private ClimbIOInputsAutoLogged climbIOInputs = new ClimbIOInputsAutoLogged();

  private ClimbVisualizer leftVisualizer = new ClimbVisualizer(ClimbSide.LEFT);
  private ClimbVisualizer rightVisualizer = new ClimbVisualizer(ClimbSide.RIGHT);

  public Climb(ClimbIO io) {
    climbIO = io;
  }

  @Override
  public void periodic() {
    climbIO.updateInputs(climbIOInputs);
    Logger.processInputs("Climb", climbIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors();
    }

    if (currentSetpoint != null) {
      climbIO.setLeftVolts(currentSetpoint.getLeftVolts());
      climbIO.setRightVolts(currentSetpoint.getRightVolts());
    }

    leftVisualizer.updateClimbAngle(climbIOInputs.leftPosition);
    rightVisualizer.updateClimbAngle(climbIOInputs.rightPosition);
  }

  /** Sets the current setpoint to null and sets the volts to 0 */
  public void stopMotors() {
    this.currentSetpoint = null;

    climbIO.setLeftVolts(0.0);
    climbIO.setRightVolts(0.0);
  }

  /** Run the climb at a setpoint voltage */
  public void runClimb(ClimbSetpoints setpoint) {
    currentSetpoint = setpoint;
  }

  /** Returns the current setpoint state of the Climb */
  @AutoLogOutput(key = "Climb/CurrentSetpoint")
  public ClimbSetpoints getCurrentSetpoint() {
    if (currentSetpoint != null) {
      return currentSetpoint;
    } else {
      return ClimbSetpoints.STOPPED;
    }
  }
}
