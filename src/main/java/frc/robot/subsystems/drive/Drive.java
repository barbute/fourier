// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.TargetingSystem;
import frc.robot.util.debugging.Alert;
import frc.robot.util.debugging.Alert.AlertType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(14.5);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(24.25);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(24.25);
  private static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED_MPS = MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS;
  // Second argument is the max accel
  public static final ModuleLimits MODULE_LIMITS =
      new ModuleLimits(MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS * 5, MAX_ANGULAR_SPEED_MPS);

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private final Translation2d[] MODULE_TRANSLATIONS = getModuleTranslations();
  private final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();

  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          });
  private SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(KINEMATICS, MODULE_TRANSLATIONS);
  private boolean areModulesOrienting = false;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private Alert gyroDisconnectAlert = new Alert("Console", ":: GYRO DISCONNECT", AlertType.ERROR);
  private Alert pathfindEnabledAlert =
      new Alert("Console", ":: PATHFINDING ENABLED", AlertType.INFO);
  private Alert debuggingModeEnabledAlert =
      new Alert("Console", ":: TUNING MODE ENABLED", AlertType.WARNING);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure setpoint generator
    setpointGenerator =
        SwerveSetpointGenerator.builder()
            .kinematics(KINEMATICS)
            .moduleLocations(MODULE_TRANSLATIONS)
            .build();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.0, 0.0, 0.0),
            new PIDConstants(1.0, 0.0, 0.0),
            MAX_LINEAR_SPEED_MPS,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig(true, true)),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Drive/Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    if (!gyroInputs.connected) {
      gyroDisconnectAlert.set(true);
    } else {
      gyroDisconnectAlert.set(false);
    }

    if (Constants.debuggingMode) {
      debuggingModeEnabledAlert.set(true);
    } else {
      debuggingModeEnabledAlert.set(false);
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    poseEstimator.update(rawGyroRotation, modulePositions);

    // TODO Update accordingly when vision is added
    TargetingSystem.getInstance()
        .updateCurrentOdometryPosition(new Pose3d(poseEstimator.getEstimatedPosition()));
    TargetingSystem.getInstance()
        .updateCurrentFilteredPosition(new Pose3d(poseEstimator.getEstimatedPosition()));
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = discretize(speeds); // Translational skew compensation
    desiredChassisSpeeds = discreteSpeeds;
    SwerveModuleState[] setpointStates = KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, MAX_LINEAR_SPEED_MPS); // Normalize speeds

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

    if (!areModulesOrienting) {
      currentSetpoint =
          setpointGenerator.generateSetpoint(MODULE_LIMITS, currentSetpoint, discreteSpeeds, 0.02);

      for (int i = 0; i < 4; i++) {
        // Optimized azimuth setpoint angles
        optimizedSetpointStates[i] =
            SwerveModuleState.optimize(currentSetpoint.moduleStates()[i], modules[i].getAngle());

        // Prevent jittering from small joystick inputs or noise
        optimizedSetpointStates[i] =
            (Math.abs(optimizedSetpointStates[i].speedMetersPerSecond / MAX_LINEAR_SPEED_MPS)
                    > 0.01)
                ? modules[i].runSetpoint(optimizedSetpointStates[i])
                : modules[i].runSetpoint(
                    new SwerveModuleState(
                        optimizedSetpointStates[i].speedMetersPerSecond, modules[i].getAngle()));

        // Run state
        modules[i].runSetpoint(optimizedSetpointStates[i]);
      }
    } else {
      for (int i = 0; i < 4; i++) {
        optimizedSetpointStates[i] =
            modules[i].runSetpoint(
                setpointStates[i]); // setDesiredState returns the optimized state
      }
    }

    // Log setpoint states
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    Logger.recordOutput("Drive/SwerveStates/DesiredSpeeds", desiredChassisSpeeds);
  }

  /** Custom method for discretizing swerve speeds */
  private ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose =
        new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            new Rotation2d(speeds.omegaRadiansPerSecond * dt * 3));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns a command to pathfind to a position */
  public Command pathfindToPose(Supplier<Pose2d> desiredPose) {
    PathConstraints pathConstraints = new PathConstraints(4.42, 3.0, 540.0, 720.0);

    Command pathCommand = AutoBuilder.pathfindToPose(desiredPose.get(), pathConstraints, 0.0, 0.0);

    return Commands.runOnce(() -> pathfindEnabledAlert.set(true))
        .andThen(pathCommand)
        .andThen(Commands.runOnce(() -> pathfindEnabledAlert.set(false)));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Drive/Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED_MPS;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED_MPS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }
}
