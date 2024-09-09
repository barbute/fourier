// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.debugging.LoggedTunableNumber;

/** LauncherIO implementation for TalonFX motor controller (Falcon500) */
public class LauncherIOTalonFX implements LauncherIO {
  private final double GEARING = 1.0 / 1.0;
  private final double RADIUS_M = 6.35 / 100;
  private final double CIRCUMFRENCE_M = 2.0 * Math.PI * RADIUS_M;

  private TalonFX topMotor = new TalonFX(43, "CTREBUS");
  private TalonFX bottomMotor = new TalonFX(44, "CTREBUS");

  private TalonFXConfiguration topConfiguration = new TalonFXConfiguration();
  private TalonFXConfiguration bottomConfiguration = new TalonFXConfiguration();

  private VelocityVoltage topVelocityVoltage = new VelocityVoltage(0.0);
  private VelocityVoltage bottomVelocityVoltage = new VelocityVoltage(0.0);

  private double topVelocitySetpointMPS = 0.0;
  private double bottomVelocitySetpointMPS = 0.0;

  private LoggedTunableNumber topFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/P", 0.026579);
  private LoggedTunableNumber topFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/I", 0.0);
  private LoggedTunableNumber topFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherTop/Feedback/D", 0.0);

  private LoggedTunableNumber bottomFeedbackP =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/P", 0.032508);
  private LoggedTunableNumber bottomFeedbackI =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/I", 0.0);
  private LoggedTunableNumber bottomFeedbackD =
      new LoggedTunableNumber("Shooter/LauncherBottom/Feedback/D", 0.0);

  public LauncherIOTalonFX() {
    topConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    topConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    bottomConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfiguration.CurrentLimits.StatorCurrentLimit = 40;
    topConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    topConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
    bottomConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomConfiguration.CurrentLimits.SupplyCurrentLimit = 40;

    topConfiguration.Voltage.PeakForwardVoltage = 12.0;
    topConfiguration.Voltage.PeakReverseVoltage = -12.0;
    bottomConfiguration.Voltage.PeakForwardVoltage = 12.0;
    bottomConfiguration.Voltage.PeakReverseVoltage = -12.0;

    topConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    bottomConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    topConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    topConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    bottomConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    topConfiguration.Slot0.kP = topFeedbackP.get();
    topConfiguration.Slot0.kI = topFeedbackI.get();
    topConfiguration.Slot0.kD = topFeedbackD.get();

    bottomConfiguration.Slot0.kP = bottomFeedbackP.get();
    bottomConfiguration.Slot0.kI = bottomFeedbackI.get();
    bottomConfiguration.Slot0.kD = bottomFeedbackD.get();

    topConfiguration.Slot0.kS = 0.0060808;
    topConfiguration.Slot0.kG = 0.0;
    topConfiguration.Slot0.kV = 0.11127;
    topConfiguration.Slot0.kA = 0.014977;

    bottomConfiguration.Slot0.kS = 0.060808;
    bottomConfiguration.Slot0.kG = 0.0;
    bottomConfiguration.Slot0.kV = 0.10994;
    bottomConfiguration.Slot0.kA = 0.014977;

    topMotor.getConfigurator().apply(topConfiguration);
    bottomMotor.getConfigurator().apply(bottomConfiguration);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        topMotor.getVelocity(),
        topMotor.getMotorVoltage(),
        topMotor.getSupplyCurrent(),
        topMotor.getDeviceTemp(),
        bottomMotor.getVelocity(),
        bottomMotor.getMotorVoltage(),
        bottomMotor.getSupplyCurrent(),
        bottomMotor.getDeviceTemp());
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.topVelocityMPS = (topMotor.getVelocity().getValueAsDouble() * CIRCUMFRENCE_M) / GEARING;
    inputs.topVelocitySetpointMPS = topVelocitySetpointMPS;
    inputs.topVelocityErrorMPS = inputs.topVelocityMPS - inputs.topVelocityErrorMPS;
    inputs.topAppliedVolts = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.topAppliedCurrentAmps = new double[] {topMotor.getSupplyCurrent().getValueAsDouble()};
    inputs.topTemperatureCelsius = new double[] {topMotor.getDeviceTemp().getValueAsDouble()};

    inputs.bottomVelocityMPS =
        (bottomMotor.getVelocity().getValueAsDouble() * CIRCUMFRENCE_M) / GEARING;
    inputs.bottomVelocitySetpointMPS = bottomVelocitySetpointMPS;
    inputs.bottomVelocityErrorMPS = inputs.bottomVelocityMPS - inputs.bottomVelocityErrorMPS;
    inputs.bottomAppliedVolts = bottomMotor.getMotorVoltage().getValueAsDouble();
    inputs.bottomAppliedCurrentAmps =
        new double[] {bottomMotor.getSupplyCurrent().getValueAsDouble()};
    inputs.bottomTemperatureCelsius = new double[] {bottomMotor.getDeviceTemp().getValueAsDouble()};

    if (Constants.debuggingMode) {
      updateTunableNumbers();
    }
  }

  @Override
  public void setTopVolts(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void setBottomVolts(double volts) {
    bottomMotor.setVoltage(volts);
  }

  @Override
  public void setTopVelocityMPS(double velocitySetpointMPS, double accelerationSetpointMPS) {
    topVelocitySetpointMPS = velocitySetpointMPS;
    topMotor.setControl(topVelocityVoltage.withVelocity(topVelocitySetpointMPS / CIRCUMFRENCE_M));
  }

  @Override
  public void setBottomVelocityMPS(double velocitySetpointMPS, double accelerationSetpointMPS) {
    bottomVelocitySetpointMPS = velocitySetpointMPS;
    bottomMotor.setControl(
        bottomVelocityVoltage.withVelocity(bottomVelocitySetpointMPS / CIRCUMFRENCE_M));
  }

  /** Method to update tunable numbers */
  private void updateTunableNumbers() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          topConfiguration.Slot0.kP = topFeedbackP.get();
          topConfiguration.Slot0.kI = topFeedbackI.get();
          topConfiguration.Slot0.kD = topFeedbackD.get();

          topMotor.getConfigurator().apply(topConfiguration);
        },
        topFeedbackP,
        topFeedbackI,
        topFeedbackD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          bottomConfiguration.Slot0.kP = bottomFeedbackP.get();
          bottomConfiguration.Slot0.kI = bottomFeedbackI.get();
          bottomConfiguration.Slot0.kD = bottomFeedbackD.get();

          bottomMotor.getConfigurator().apply(bottomConfiguration);
        },
        bottomFeedbackP,
        bottomFeedbackI,
        bottomFeedbackD);
  }
}
