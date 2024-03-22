// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

/** Physics sim implementation of IndexerIO */
public class IndexerIOSim implements IndexerIO {
  private static final double LOOP_PERIOD_SECS = 0.02;
  private final double GEAR_RATIO = 5.0 / 1.0;

  private FlywheelSim indexerMotor = new FlywheelSim(DCMotor.getNEO(1), GEAR_RATIO, 0.0002);

  private double appliedVolts = 0.0;
  // Switch the beam break state in sim
  private LoggedDashboardBoolean beamBreakSensor =
      new LoggedDashboardBoolean("Indexer/SimBeamBreak", false);

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerMotor.update(LOOP_PERIOD_SECS);

    inputs.velocityRPM = indexerMotor.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedCurrentAmps = new double[] {indexerMotor.getCurrentDrawAmps()};
    inputs.temperatureCelsius = new double[] {0.0};

    inputs.beamBroken = beamBreakSensor.get();
  }

  @Override
  public void setVolts(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    indexerMotor.setInputVoltage(appliedVolts);
  }
}
