// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.util.debugging.LoggedTunableNumber;

/** Physics sim implementation of LauncherIO */
public class LauncherIOSim implements LauncherIO {
  private final double LOOP_PERIOD_SECS = 0.02;
  private final double GEARING = 1.0 / 1.0;
  private final double RADIUS_M = 6.35 / 100;
  private final double CIRCUMFRENCE_M = 2.0 * Math.PI * RADIUS_M;

  private FlywheelSim topMotor = new FlywheelSim(DCMotor.getFalcon500(1), GEARING, 0.009);
  private FlywheelSim bottomMotor = new FlywheelSim(DCMotor.getFalcon500(1), GEARING, 0.009);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;

  private PIDController topFeedback = new PIDController(5.9, 0.0, 0.0);
  private SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(0.0, 0.237);
  private PIDController bottomFeedback = new PIDController(5.9, 0.0, 0.0);
  private SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(0.0, 0.237);

  private LoggedTunableNumber topFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/P", topFeedback.getP());
  private LoggedTunableNumber topFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/I", topFeedback.getI());
  private LoggedTunableNumber topFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/D", topFeedback.getD());

  private LoggedTunableNumber bottomFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/P", bottomFeedback.getP());
  private LoggedTunableNumber bottomFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/I", bottomFeedback.getI());
  private LoggedTunableNumber bottomFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/D", bottomFeedback.getD());

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    topMotor.update(LOOP_PERIOD_SECS);
    bottomMotor.update(LOOP_PERIOD_SECS);

    inputs.topVelocityMPS = (topMotor.getAngularVelocityRPM() * CIRCUMFRENCE_M) / 60.0;
    inputs.topVelocitySetpointMPS = topFeedback.getSetpoint();
    inputs.topVelocityErrorMPS = topFeedback.getPositionError();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topAppliedCurrentAmps = new double[] {topMotor.getCurrentDrawAmps()};
    inputs.topTemperatureCelsius = new double[] {0.0};

    inputs.bottomVelocityMPS = (bottomMotor.getAngularVelocityRPM() * CIRCUMFRENCE_M) / 60.0;
    inputs.bottomVelocitySetpointMPS = bottomFeedback.getSetpoint();
    inputs.bottomVelocityErrorMPS = bottomFeedback.getPositionError();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomAppliedCurrentAmps = new double[] {bottomMotor.getCurrentDrawAmps()};
    inputs.bottomTemperatureCelsius = new double[] {0.0};

    if (Constants.debuggingMode) {
      updateTunableNumbers();
    }
  }

  @Override
  public void setTopVolts(double volts) {
    topAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    topMotor.setInputVoltage(topAppliedVolts);
  }

  @Override
  public void setBottomVolts(double volts) {
    bottomAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    bottomMotor.setInputVoltage(bottomAppliedVolts);
  }

  @Override
  public void setTopVelocityMPS(double velocitySetpointMPS, double accelerationSetpointMPS) {
    double topFeedbackOutput =
        topFeedback.calculate(
            (topMotor.getAngularVelocityRPM() * CIRCUMFRENCE_M) / 60.0, velocitySetpointMPS);
    double topFeedforwardOutput =
        topFeedforward.calculate(velocitySetpointMPS, accelerationSetpointMPS);

    double topCombinedOutput = topFeedbackOutput + topFeedforwardOutput;

    setTopVolts(topCombinedOutput);
  }

  @Override
  public void setBottomVelocityMPS(double velocitySetpointMPS, double accelerationSetpointMPS) {
    double bottomFeedbackOutput =
        bottomFeedback.calculate(
            (bottomMotor.getAngularVelocityRPM() * CIRCUMFRENCE_M) / 60.0, velocitySetpointMPS);
    double bottomFeedforwardOutput =
        bottomFeedforward.calculate(velocitySetpointMPS, accelerationSetpointMPS);

    double bottomCombinedOutput = bottomFeedbackOutput + bottomFeedforwardOutput;

    setBottomVolts(bottomCombinedOutput);
  }

  /** Method to update tunable numbers */
  private void updateTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          topFeedback.setP(topFeedbackP.get());
          topFeedback.setI(topFeedbackI.get());
          topFeedback.setD(topFeedbackD.get());
        },
        topFeedbackP,
        topFeedbackI,
        topFeedbackD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          bottomFeedback.setP(bottomFeedbackP.get());
          bottomFeedback.setI(bottomFeedbackI.get());
          bottomFeedback.setD(bottomFeedbackD.get());
        },
        bottomFeedbackP,
        bottomFeedbackI,
        bottomFeedbackD);
  }
}
