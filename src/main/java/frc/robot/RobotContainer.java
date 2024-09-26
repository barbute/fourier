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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbSetpoints;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
// import frc.robot.subsystems.climb.ClimbIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerSetpoints;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeSetpoints;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterSetpoints;
import frc.robot.subsystems.shooter.TargetingSystem;
import frc.robot.subsystems.shooter.angler.AnglerIO;
import frc.robot.subsystems.shooter.angler.AnglerIOSim;
import frc.robot.subsystems.shooter.angler.AnglerIOSparkMax;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOSim;
import frc.robot.subsystems.shooter.launcher.LauncherIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Camera;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.math.AllianceFlipUtil;
import java.util.ArrayList;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Drive robotDrive;
  private Shooter robotShooter;
  private Intake robotIntake;
  private Indexer robotIndexer;
  private Vision robotVision;
  private Climb robotClimb;

  private ArrayList<VisionIO> realVisionIOs = new ArrayList<VisionIO>();
  private ArrayList<VisionIO> simVisionIOs = new ArrayList<VisionIO>();
  private ArrayList<VisionIO> replayVisionIOs = new ArrayList<VisionIO>();

  private Chassis robotChassis;

  private final CommandXboxController pilotController = new CommandXboxController(0);
  private final CommandXboxController copilotController = new CommandXboxController(1);

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        robotDrive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        robotShooter = new Shooter(new AnglerIOSparkMax(), new LauncherIOTalonFX());
        robotIntake = new Intake(new IntakeIOSparkMax());
        robotIndexer = new Indexer(new IndexerIOSparkMax());
        realVisionIOs.add(new VisionIOPhoton(Camera.FRONT_LEFT_0));
        realVisionIOs.add(new VisionIOPhoton(Camera.FRONT_RIGHT_1));
        robotVision = new Vision(realVisionIOs);
        // robotYoshi = new Yoshi(new YoshiIOSparkMax());
        robotClimb = new Climb(new ClimbIOSparkMax());
        // robotClimb = new Climb(new ClimbIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        robotDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        robotShooter = new Shooter(new AnglerIOSim(), new LauncherIOSim());
        robotIntake = new Intake(new IntakeIOSim());
        robotIndexer = new Indexer(new IndexerIOSim());
        simVisionIOs.add(new VisionIOSim(Camera.FRONT_LEFT_0, () -> robotDrive.getOdometryPose()));
        simVisionIOs.add(new VisionIOSim(Camera.FRONT_RIGHT_1, () -> robotDrive.getOdometryPose()));
        robotVision = new Vision(simVisionIOs);
        robotClimb = new Climb(new ClimbIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        robotDrive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        robotShooter = new Shooter(new AnglerIO() {}, new LauncherIO() {});
        robotIntake = new Intake(new IntakeIO() {});
        robotIndexer = new Indexer(new IndexerIO() {});
        replayVisionIOs.add(new VisionIO() {});
        replayVisionIOs.add(new VisionIO() {});
        robotVision = new Vision(replayVisionIOs);
        robotClimb = new Climb(new ClimbIO() {});
        break;
    }

    robotChassis = new Chassis(robotDrive, robotVision);

    // Set up auto routines
    NamedCommands.registerCommand("Print", new PrintCommand("Hello, world!"));
    NamedCommands.registerCommand(
        "stow-start",
        new InstantCommand(
            () -> {
              robotIndexer.runIndexer(IndexerSetpoints.STOW);
              robotIntake.runIntake(IntakeSetpoints.INTAKE);
              robotShooter.runShooter(ShooterSetpoints.INTAKE);
            }));
    NamedCommands.registerCommand(
        "aim-subwoofer",
        new InstantCommand(
                () -> {
                  robotShooter.runShooter(ShooterSetpoints.SUBWOOFER);
                })
            .repeatedly()
            .withTimeout(2.5));
    NamedCommands.registerCommand(
        "shooter-stop",
        new InstantCommand(
            () -> {
              robotShooter.runShooter(ShooterSetpoints.STOPPED);
            }));
    NamedCommands.registerCommand(
        "rollers-stop",
        new InstantCommand(
            () -> {
              robotIndexer.runIndexer(IndexerSetpoints.STOPPED);
              robotIntake.runIntake(IntakeSetpoints.STOPPED);
            }));
    NamedCommands.registerCommand(
        "fire",
        Commands.runEnd(
                () -> {
                  robotIndexer.runIndexer(IndexerSetpoints.SPEAKER_SHOOT);
                  robotIntake.runIntake(IntakeSetpoints.SPEAKER_SHOOT);
                },
                () -> {
                  robotIndexer.runIndexer(IndexerSetpoints.STOPPED);
                  robotIntake.runIntake(IntakeSetpoints.STOPPED);
                },
                robotIndexer,
                robotIntake)
            .repeatedly()
            .withTimeout(1.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        robotDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", robotDrive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", robotDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysID All", robotDrive.runAllCharacterization());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new Trigger(
            () ->
                robotShooter.atGoal()
                    && (robotShooter.getCurrentSetpoint() == ShooterSetpoints.AIM
                        || robotShooter.getCurrentSetpoint() == ShooterSetpoints.AMP
                        || robotShooter.getCurrentSetpoint() == ShooterSetpoints.SUBWOOFER
                        || robotShooter.getCurrentSetpoint() == ShooterSetpoints.FEEDER))
        .whileTrue(
            Commands.runOnce(
                () -> copilotController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.2)))
        .whileFalse(
            Commands.runOnce(
                () ->
                    copilotController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));

    robotDrive.acceptTeleroperatedInputs(
        () -> -pilotController.getLeftY(),
        () -> -pilotController.getLeftX(),
        () -> -pilotController.getRightX());
    robotDrive.setDefaultCommand(
        Commands.run(() -> robotDrive.setDriveState(DriveState.TELEOPERATED), robotDrive));

    pilotController
        .a()
        .whileTrue(
            DriveCommands.automaticHeading(
                robotDrive,
                () -> -pilotController.getLeftY(),
                () -> -pilotController.getLeftX(),
                () ->
                    TargetingSystem.getInstance()
                        .calculateOptimalHeading(
                            new Pose3d(
                                AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening),
                                new Rotation3d()))
                        .rotateBy(Rotation2d.fromDegrees(180.0))))
        .onFalse(DriveCommands.stopDrive(robotDrive));

    pilotController
        .y()
        .onTrue(
            Commands.runOnce(
                () ->
                    robotDrive.setPose(
                        new Pose2d(
                            robotDrive.getOdometryPose().getTranslation(), new Rotation2d())),
                robotDrive));

    pilotController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  robotDrive.setPose(new Pose2d(1.37, 5.54, Rotation2d.fromDegrees(0.0)));
                },
                robotDrive));

    // --------------------------------------------------

    // STOW
    copilotController
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  robotIntake.runIntake(IntakeSetpoints.INTAKE);
                  robotIndexer.runIndexer(IndexerSetpoints.STOW);
                  robotShooter.runShooter(ShooterSetpoints.INTAKE);
                },
                () -> {
                  robotIntake.runIntake(IntakeSetpoints.STOPPED);
                  robotIndexer.runIndexer(IndexerSetpoints.STOPPED);
                  robotShooter.runShooter(ShooterSetpoints.TRAVERSAL);
                },
                robotIntake,
                robotIndexer,
                robotShooter));

    // OUTTAKE
    copilotController
        .rightBumper()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  robotIntake.runIntake(IntakeSetpoints.OUTTAKE);
                  robotIndexer.runIndexer(IndexerSetpoints.OUTTAKE);
                },
                () -> {
                  robotIntake.runIntake(IntakeSetpoints.STOPPED);
                  robotIndexer.runIndexer(IndexerSetpoints.STOPPED);
                },
                robotIntake,
                robotIndexer));

    // AMP FIRE
    copilotController
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  robotIntake.runIntake(IntakeSetpoints.AMP_SHOOT);
                  robotIndexer.runIndexer(IndexerSetpoints.AMP_SHOOT);
                },
                () -> {
                  robotIntake.runIntake(IntakeSetpoints.STOPPED);
                  robotIndexer.runIndexer(IndexerSetpoints.STOPPED);
                },
                robotIntake,
                robotIndexer));

    // FIRE
    copilotController
        .rightTrigger()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  robotIntake.runIntake(IntakeSetpoints.SPEAKER_SHOOT);
                  robotIndexer.runIndexer(IndexerSetpoints.SPEAKER_SHOOT);
                },
                () -> {
                  robotIntake.runIntake(IntakeSetpoints.STOPPED);
                  robotIndexer.runIndexer(IndexerSetpoints.STOPPED);
                },
                robotIntake,
                robotIndexer));

    // SUBWOOFER
    copilotController
        .y()
        .whileTrue(
            Commands.runOnce(
                () -> robotShooter.runShooter(ShooterSetpoints.SUBWOOFER), robotShooter))
        .whileFalse(
            Commands.runOnce(
                () -> robotShooter.runShooter(ShooterSetpoints.TRAVERSAL), robotShooter));

    // PASS
    copilotController
        .x()
        .whileTrue(
            Commands.runOnce(() -> robotShooter.runShooter(ShooterSetpoints.FEEDER), robotShooter))
        .whileFalse(
            Commands.runOnce(
                () -> robotShooter.runShooter(ShooterSetpoints.TRAVERSAL), robotShooter));

    // AIM
    copilotController
        .a()
        .whileTrue(
            Commands.runOnce(() -> robotShooter.runShooter(ShooterSetpoints.AIM), robotShooter))
        .whileFalse(
            Commands.runOnce(
                () -> robotShooter.runShooter(ShooterSetpoints.STOPPED), robotShooter));

    // AMP
    copilotController
        .b()
        .whileTrue(Commands.run(() -> robotShooter.runShooter(ShooterSetpoints.AMP), robotShooter))
        .whileFalse(
            Commands.runOnce(
                () -> robotShooter.runShooter(ShooterSetpoints.STOPPED), robotShooter));

    copilotController
        .povUp()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  robotShooter.runShooter(ShooterSetpoints.MANUAL_UP);
                },
                () -> {
                  robotShooter.runShooter(ShooterSetpoints.STOPPED);
                },
                robotShooter));

    copilotController
        .povDown()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  robotShooter.runShooter(ShooterSetpoints.MANUAL_DOWN);
                },
                () -> {
                  robotShooter.runShooter(ShooterSetpoints.STOPPED);
                },
                robotShooter));

    copilotController
        .povLeft()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  robotClimb.runClimb(ClimbSetpoints.BOTH_IN);
                  robotShooter.runShooter(ShooterSetpoints.CLIMB);
                },
                () -> {
                  robotClimb.stopMotors();
                  robotShooter.runShooter(ShooterSetpoints.HOLD);
                },
                robotClimb,
                robotShooter));

    copilotController
        .povRight()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  robotClimb.runClimb(ClimbSetpoints.BOTH_OUT);
                  robotShooter.runShooter(ShooterSetpoints.CLIMB);
                },
                () -> {
                  robotClimb.stopMotors();
                  robotShooter.runShooter(ShooterSetpoints.HOLD);
                },
                robotClimb,
                robotShooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Returns the Chassis instance to run in robotPeriodic() */
  public Chassis getChassisInstance() {
    return robotChassis;
  }
}
