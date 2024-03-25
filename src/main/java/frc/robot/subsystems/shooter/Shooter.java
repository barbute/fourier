// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.util.debugging.Alert;
import frc.robot.util.debugging.Alert.AlertType;
import frc.robot.util.debugging.LoggedTunableNumber;
import frc.robot.util.math.LinearProfile;
import frc.robot.util.math.ScrewArmFeedforward;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public enum ShooterClosedLoopSetpoints {
    INTAKE(() -> Rotation2d.fromDegrees(45.0), () -> 0.0, () -> 0.0),
    TRAVERSAL(() -> Rotation2d.fromDegrees(30.0), () -> 10.0, () -> 10.0),
    CLIMB(() -> Rotation2d.fromDegrees(25.0), () -> 0.0, () -> 0.0),
    // TODO Replace with targeting system angle when done
    AIM(() -> Rotation2d.fromDegrees(0.0), () -> 38.0, () -> 38.0),
    SUBWOOFER(() -> Rotation2d.fromDegrees(57.0), () -> 38.0, () -> 38.0),
    AMP(() -> Rotation2d.fromDegrees(54.0), () -> -4.5, () -> 12.0),
    FEEDER(() -> Rotation2d.fromDegrees(50.0), () -> 30.0, () -> 30.0),
    STOPPED(
        () ->
            Rotation2d.fromDegrees(
                new LoggedTunableNumber("Shooter/Angler/SetpointDebuggingDegrees", 0.0).get()),
        () -> new LoggedTunableNumber("Shooter/Launcher/TopDebuggingMPS", 0.0).get(),
        () -> new LoggedTunableNumber("Shooter/Launcher/BottomDebuggingMPS", 0.0).get()),
    CUSTOM(() -> Rotation2d.fromDegrees(0.0), () -> 0.0, () -> 0.0);

    private Supplier<Rotation2d> setpointPosition;
    private DoubleSupplier topSetpointVelocityMPS;
    private DoubleSupplier bottomSetpointVelocityMPS;

    ShooterClosedLoopSetpoints(
        Supplier<Rotation2d> position,
        DoubleSupplier topVelocityMPS,
        DoubleSupplier bottomVelocityMPS) {
      this.setpointPosition = position;
      this.topSetpointVelocityMPS = topVelocityMPS;
      this.bottomSetpointVelocityMPS = bottomVelocityMPS;
    }

    public Rotation2d getPosition() {
      return this.setpointPosition.get();
    }

    public double getTopVelocityMPS() {
      return this.topSetpointVelocityMPS.getAsDouble();
    }

    public double getBottomVelocityMPS() {
      return this.bottomSetpointVelocityMPS.getAsDouble();
    }
  }

  private ShooterClosedLoopSetpoints currentClosedLoopSetpoint = null;

  private Rotation2d currentPositionSetpoint = null;
  private Double currentTopFlywheelVelocitySetpointMPS = null;
  private Double currentBottomFlywheelVelocitySetpointMPS = null;

  private AnglerIO anglerIO;
  private AnglerIOInputsAutoLogged anglerIOInputs = new AnglerIOInputsAutoLogged();

  private LauncherIO launcherIO;
  private LauncherIOInputsAutoLogged launcherIOInputs = new LauncherIOInputsAutoLogged();

  private ProfiledPIDController anglerFeedback;
  private ScrewArmFeedforward anglerFeedforward;

  private LoggedTunableNumber anglerFeedbackP;
  private LoggedTunableNumber anglerFeedbackI;
  private LoggedTunableNumber anglerFeedbackD;
  private LoggedTunableNumber anglerFeedbackV;
  private LoggedTunableNumber anglerFeedbackA;

  private LinearProfile topFlywheelProfile = new LinearProfile(50.0, 0.02);
  private LinearProfile bottomFlywheelProfile = new LinearProfile(50.0, 0.02);

  private boolean angleEncoderCalibrated = false;
  private Rotation2d angleOffset = new Rotation2d();
  private Rotation2d currentPosition = null;

  private Alert angleEncoderCalibratedSuccessAlert =
      new Alert("Console", "ANGLER ENCODER CALIBRATION SUCCESSFULL", AlertType.INFO);
  private Alert angleEncoderCalibratedFailedAlert =
      new Alert("Console", "ANGLER ENCODER CALIBRATION FAILED", AlertType.ERROR);

  public Shooter(AnglerIO anglerIO, LauncherIO launcherIO) {
    this.anglerIO = anglerIO;
    this.launcherIO = launcherIO;

    switch (Constants.currentMode) {
      case REAL:
        anglerFeedback =
            new ProfiledPIDController(
                0.49, 2.0, 0.018, new TrapezoidProfile.Constraints(1000.0, 1000.0));
        anglerFeedforward = new ScrewArmFeedforward(0.2, 0.0);
        break;
      case SIM:
        anglerFeedback =
            new ProfiledPIDController(0.1, 0.0, 0.5, new TrapezoidProfile.Constraints(50.0, 50.0));
        anglerFeedforward = new ScrewArmFeedforward(0.0, 0.0);
        break;
      case REPLAY:
        anglerFeedback =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        anglerFeedforward = new ScrewArmFeedforward(0.0, 0.0);
        break;
      default:
        anglerFeedback =
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
        anglerFeedforward = new ScrewArmFeedforward(0.0, 0.0);
        break;
    }

    anglerFeedbackP = new LoggedTunableNumber("Shooter/Angler/FeedbackP", anglerFeedback.getP());
    anglerFeedbackI = new LoggedTunableNumber("Shooter/Angler/FeedbackI", anglerFeedback.getI());
    anglerFeedbackD = new LoggedTunableNumber("Shooter/Angler/FeedbackD", anglerFeedback.getD());
    anglerFeedbackV =
        new LoggedTunableNumber(
            "Shooter/Angler/FeedbackV", anglerFeedback.getConstraints().maxVelocity);
    anglerFeedbackA =
        new LoggedTunableNumber(
            "Shooter/Angler/FeedbackA", anglerFeedback.getConstraints().maxAcceleration);

    resetAnglerFeedback();
    anglerFeedback.setTolerance(0.25);
    anglerFeedback.setIZone(20.0);
    anglerFeedback.setIntegratorRange(-0.5, 0.5);
  }

  @Override
  public void periodic() {
    anglerIO.updateInputs(anglerIOInputs);
    Logger.processInputs("Shooter/Angler", anglerIOInputs);
    launcherIO.updateInputs(launcherIOInputs);
    Logger.processInputs("Shooter/Launcher", launcherIOInputs);

    if (DriverStation.isDisabled()) {
      stopMotors();
    }

    if (!angleEncoderCalibrated) {
      for (int i = 0; i < 100; i++) {
        if (anglerIOInputs.anglerDutyCycleFrequency == 955) {
          angleOffset =
              Rotation2d.fromRadians(
                      MathUtil.inputModulus(
                          anglerIOInputs.absolutePosition.getRadians(), 0, 2.0 * Math.PI))
                  .minus(anglerIOInputs.relativePosition);
          angleEncoderCalibrated = true;

          angleEncoderCalibratedFailedAlert.set(false);
          angleEncoderCalibratedSuccessAlert.set(true);
          break;
        }
        if (i == 99) {
          angleOffset =
              Rotation2d.fromRadians(
                      MathUtil.inputModulus(
                          anglerIOInputs.absolutePosition.getRadians(), 0, 2.0 * Math.PI))
                  .minus(anglerIOInputs.relativePosition);
          angleEncoderCalibrated = true;

          angleEncoderCalibratedFailedAlert.set(true);
          angleEncoderCalibratedSuccessAlert.set(false);
          break;
        }
        // TODO Initialize visualizer here
      }
      currentPosition = anglerIOInputs.relativePosition.plus(angleOffset);

      resetAnglerFeedback();
    }

    currentPosition = anglerIOInputs.relativePosition.plus(angleOffset);

    if (currentPositionSetpoint != null) {
      Rotation2d positionSetpoint =
          Rotation2d.fromDegrees(MathUtil.clamp(currentPositionSetpoint.getDegrees(), 26.5, 57.0));

      double anglerFeedbackOutput =
          anglerFeedback.calculate(currentPosition.getDegrees(), positionSetpoint.getDegrees());
      double anglerFeedforwardOutput =
          anglerFeedforward.calculate(currentPosition, positionSetpoint);

      double anglerCombinedOutput = anglerFeedbackOutput + anglerFeedforwardOutput;

      anglerIO.setVolts(anglerCombinedOutput);
    }

    if (currentTopFlywheelVelocitySetpointMPS != null) {
      launcherIO.setTopVelocityMPS(
          topFlywheelProfile.calculateSetpoint(), topFlywheelProfile.getCurrentAcceleration());
    }
    if (currentBottomFlywheelVelocitySetpointMPS != null) {
      launcherIO.setBottomVelocityMPS(
          bottomFlywheelProfile.calculateSetpoint(),
          bottomFlywheelProfile.getCurrentAcceleration());
    }

    // TODO Update visualizer here

    if (Constants.debuggingMode) {
      // Only updates angler PIDs, launcher runs in its own IO implementation
      updateTunableNumbers();
    }
  }

  /** Method to update tunable numbers */
  private void updateTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          anglerFeedback.setP(anglerFeedbackP.get());
          anglerFeedback.setI(anglerFeedbackP.get());
          anglerFeedback.setD(anglerFeedbackP.get());
          anglerFeedback.setConstraints(
              new TrapezoidProfile.Constraints(anglerFeedbackV.get(), anglerFeedbackA.get()));
        },
        anglerFeedbackP,
        anglerFeedbackI,
        anglerFeedbackD,
        anglerFeedbackV,
        anglerFeedbackA);
  }

  /** Sets the current setpoint to null and sets all motor voltages to 0 */
  public void stopMotors() {
    currentClosedLoopSetpoint = ShooterClosedLoopSetpoints.STOPPED;

    currentPositionSetpoint = null;
    currentTopFlywheelVelocitySetpointMPS = null;
    currentBottomFlywheelVelocitySetpointMPS = null;

    anglerIO.setVolts(0.0);
    launcherIO.setTopVolts(0.0);
    launcherIO.setBottomVolts(0.0);
  }

  /** Set the motors using a pre-defined setpoint */
  private void setMotors(ShooterClosedLoopSetpoints setpoint) {
    setAnglerPosition(setpoint.getPosition());
    setLauncherVelocityMPS(setpoint.getTopVelocityMPS(), setpoint.getBottomVelocityMPS());
  }

  /** Set the angler position */
  private void setAnglerPosition(Rotation2d anglerSetpoint) {
    currentPositionSetpoint = anglerSetpoint;

    if (currentPositionSetpoint != null) {
      DoubleSupplier errorDegrees =
          () -> Math.abs(currentPositionSetpoint.minus(getCurrentPosition()).getDegrees());
      Logger.recordOutput("Shooter/AnglerReset/ErrorDegrees", errorDegrees.getAsDouble());

      if (currentPositionSetpoint != null && errorDegrees.getAsDouble() > 2.0) {
        resetAnglerFeedback();
      }
    }
  }

  /** Set the launcher closed-loop velocity in MPS */
  private void setLauncherVelocityMPS(double topSetpointMPS, double bottomSetpointMPS) {
    currentTopFlywheelVelocitySetpointMPS = topSetpointMPS;
    currentBottomFlywheelVelocitySetpointMPS = bottomSetpointMPS;

    if (currentTopFlywheelVelocitySetpointMPS != null) {
      topFlywheelProfile.setGoal(
          currentTopFlywheelVelocitySetpointMPS, launcherIOInputs.topVelocityMPS);
    }
    if (currentBottomFlywheelVelocitySetpointMPS != null) {
      bottomFlywheelProfile.setGoal(
          currentBottomFlywheelVelocitySetpointMPS, launcherIOInputs.bottomVelocityMPS);
    }
  }

  /** Reset the profile for the angler feedback controller */
  public void resetAnglerFeedback() {
    anglerFeedback.reset(getCurrentPosition().getDegrees(), 0.0);
  }

  /** Returns a command to run the shooter motors to a preset setpoint */
  public Command runShooter(ShooterClosedLoopSetpoints setpoint) {
    return new FunctionalCommand(
        () -> {
          setMotors(setpoint);
        },
        () -> {},
        (interrupted) -> {},
        () -> atSetpoint(),
        this);
  }

  /** Returns the current setpoint of the Shooter */
  @AutoLogOutput(key = "Shooter/CurrentSetpoint")
  public ShooterClosedLoopSetpoints getCurrentSetpoint() {
    return currentClosedLoopSetpoint;
  }

  /**
   * Returns whether or not all of the controllers are at their setpoint and within error tolerance
   */
  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return getAnglerPositionError().getDegrees() < 0.6
        && getTopLauncherErrorMPS() < 1.0
        && getBottomLauncherErrorMPS() < 1.0;
  }

  /** Returns the current position of the angler */
  @AutoLogOutput(key = "Shooter/Angler/Position")
  public Rotation2d getCurrentPosition() {
    if (currentPosition != null) {
      return currentPosition;
    } else {
      return new Rotation2d();
    }
  }

  /** Returns the anglular-position error of the angler */
  @AutoLogOutput(key = "Shooter/Angler/Error")
  public Rotation2d getAnglerPositionError() {
    return Rotation2d.fromDegrees(anglerFeedback.getPositionError());
  }

  /** Returns the angular-velocity error of the top flywheel */
  @AutoLogOutput(key = "Shooter/Launcher/TopErrorMPS")
  public double getTopLauncherErrorMPS() {
    return launcherIOInputs.topVelocityErrorMPS;
  }

  /** Returns the angular-velocity error of the bottom flywheel */
  @AutoLogOutput(key = "Shooter/Launcher/BottomErrorMPS")
  public double getBottomLauncherErrorMPS() {
    return launcherIOInputs.bottomVelocityErrorMPS;
  }
}
