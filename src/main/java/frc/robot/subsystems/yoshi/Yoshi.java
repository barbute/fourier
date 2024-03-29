// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.yoshi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.debugging.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Yoshi extends SubsystemBase {
  public enum YoshiSetpoints {
    INTAKE(() -> Rotation2d.fromDegrees(-48.0), () -> -12.0),
    OUTTAKE(() -> Rotation2d.fromDegrees(-48.0), () -> 12.0),
    IDLE(() -> Rotation2d.fromDegrees(90.0), () -> 0.0),
    CUSTOM(
        () ->
            Rotation2d.fromDegrees(
                new LoggedTunableNumber("Yoshi/Pivot/SetpointDebuggingDegrees", 0.0).get()),
        () -> new LoggedTunableNumber("Yoshi/Flywheel/SetpointDebuggingVolts", 0.0).get()),
    // HOLD and STOPPED are handled separately in setMotors(), so the setpoints are set to zero
    HOLD(() -> Rotation2d.fromDegrees(0.0), () -> 0.0),
    STOPPED(() -> Rotation2d.fromDegrees(0.0), () -> 0.0);

    private Supplier<Rotation2d> setpointPosition;
    private DoubleSupplier setpointVolts;

    YoshiSetpoints(Supplier<Rotation2d> position, DoubleSupplier volts) {
      this.setpointPosition = position;
      this.setpointVolts = volts;
    }

    public Rotation2d getPosition() {
      return this.setpointPosition.get();
    }

    public double getVolts() {
      return this.setpointVolts.getAsDouble();
    }
  }

  private YoshiSetpoints currentYoshiSetpoint = null;

  private Rotation2d currentPositionSetpoint = null;
  private Double currentFlywheelSetpointVolts = null;

  private YoshiIO yoshiIO;
  private YoshiIOInputsAutoLogged yoshiIOInputs = new YoshiIOInputsAutoLogged();

  private ProfiledPIDController pivotFeedback;
  private ArmFeedforward pivotFeedforward;

  private LoggedTunableNumber pivotFeedbackP;
  private LoggedTunableNumber pivotFeedbackI;
  private LoggedTunableNumber pivotFeedbackD;
  private LoggedTunableNumber pivotFeedbackV;
  private LoggedTunableNumber pivotFeedbackA;

  public Yoshi(YoshiIO io) {
    yoshiIO = io;

    switch (Constants.currentMode) {
      case REAL:
        pivotFeedback =
            new ProfiledPIDController(
                0.1, 0.0, 0.0, new TrapezoidProfile.Constraints(1600.0, 800.0));
        pivotFeedforward = new ArmFeedforward(0.0, 0.4, 0.0);
        break;
      case SIM:
        pivotFeedback =
            new ProfiledPIDController(
                2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1000.0, 500.0));
        pivotFeedforward = new ArmFeedforward(0.0, 0.4, 0.0);
        break;
      case REPLAY:
        pivotFeedback =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        pivotFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
        break;
      default:
        pivotFeedback =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        pivotFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
        break;
    }

    pivotFeedbackP = new LoggedTunableNumber("Yoshi/Pivot/FeedbackP", pivotFeedback.getP());
    pivotFeedbackI = new LoggedTunableNumber("Yoshi/Pivot/FeedbackI", pivotFeedback.getI());
    pivotFeedbackD = new LoggedTunableNumber("Yoshi/Pivot/FeedbackD", pivotFeedback.getD());
    pivotFeedbackV =
        new LoggedTunableNumber(
            "Yoshi/Pivot/FeedbackV", pivotFeedback.getConstraints().maxVelocity);
    pivotFeedbackA =
        new LoggedTunableNumber(
            "Yoshi/Pivot/FeedbackA", pivotFeedback.getConstraints().maxAcceleration);

    pivotFeedback.setTolerance(0.1);
  }

  @Override
  public void periodic() {
    yoshiIO.updateInputs(yoshiIOInputs);
    Logger.processInputs("Yoshi", yoshiIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors();
    }

    if (currentPositionSetpoint != null) {
      Rotation2d positionSetpoint =
          Rotation2d.fromDegrees(
              MathUtil.clamp(currentPositionSetpoint.getDegrees(), -38.0, 110.0));

      double pivotFeedbackOutput =
          pivotFeedback.calculate(getCurrentPosition().getDegrees(), positionSetpoint.getDegrees());
      double pivotFeedforwardOutput =
          pivotFeedforward.calculate(
              Math.toRadians(pivotFeedback.getSetpoint().position),
              pivotFeedback.getSetpoint().velocity);

      double pivotCombinedOutput = (pivotFeedbackOutput + pivotFeedforwardOutput) * -1.0;

      yoshiIO.setPivotVolts(pivotCombinedOutput);
    }

    if (currentFlywheelSetpointVolts != null) {
      yoshiIO.setFlywheelVolts(currentFlywheelSetpointVolts);
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
          pivotFeedback.setP(pivotFeedbackP.get());
          pivotFeedback.setI(pivotFeedbackI.get());
          pivotFeedback.setD(pivotFeedbackD.get());
          pivotFeedback.setConstraints(
              new TrapezoidProfile.Constraints(pivotFeedbackV.get(), pivotFeedbackA.get()));
        },
        pivotFeedbackP,
        pivotFeedbackI,
        pivotFeedbackD,
        pivotFeedbackV,
        pivotFeedbackA);
  }

  /** Sets the current setpoint to null and sets all motor voltages to 0 */
  public void stopMotors() {
    // TODO Probs should look into this lmao
    currentYoshiSetpoint = YoshiSetpoints.HOLD;

    currentPositionSetpoint = null;
    currentFlywheelSetpointVolts = null;

    yoshiIO.setPivotVolts(0.0);
    yoshiIO.setFlywheelVolts(0.0);
  }

  /** Set the motors using a pre-defined setpoint */
  public void setMotors(YoshiSetpoints setpoint) {
    currentYoshiSetpoint = setpoint;

    if (currentYoshiSetpoint == YoshiSetpoints.HOLD) {
      setPivotPosition(getCurrentPosition());
      setFlywheelVolts(currentFlywheelSetpointVolts);
    } else if (currentYoshiSetpoint == YoshiSetpoints.STOPPED) {
      stopMotors();
    } else {
      setPivotPosition(currentYoshiSetpoint.getPosition());
      setFlywheelVolts(currentYoshiSetpoint.getVolts());
    }
  }

  /** Set the pivot position */
  private void setPivotPosition(Rotation2d pivotSetpoint) {
    currentPositionSetpoint = pivotSetpoint;

    if (currentPositionSetpoint != null) {
      pivotFeedback.setGoal(currentPositionSetpoint.getDegrees());
      pivotFeedback.reset(getCurrentPosition().getDegrees());
    }
  }

  /** Set the voltage of the flywheel */
  private void setFlywheelVolts(double flywheelSetpointVolts) {
    currentFlywheelSetpointVolts = flywheelSetpointVolts;
  }

  /** Returns the current setpoint of the Yoshi */
  @AutoLogOutput(key = "Yoshi/CurrentSetpoint")
  public YoshiSetpoints getCurrentSetpoint() {
    return currentYoshiSetpoint;
  }

  /** Returns the current position of the angler */
  @AutoLogOutput(key = "Yoshi/Pivot/Position")
  public Rotation2d getCurrentPosition() {
    return yoshiIOInputs.pivotPosition;
  }

  /** Returns the pivot position error in degrees */
  @AutoLogOutput(key = "Yoshi/Pivot/ErrorDegrees")
  public double getPivotErrorDegrees() {
    return pivotFeedback.getPositionError();
  }

  /** Returns whether or not the yoshi is at the desired setpoints */
  @AutoLogOutput(key = "Yoshi/AtSetpoint")
  public boolean atSetpoint() {
    return getPivotErrorDegrees() < 0.1;
  }
}
