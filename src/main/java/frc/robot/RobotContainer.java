// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import static frc.robot.Constants.OPERATOR_CONSTANTS.SLOW_SPEED_LIMITER;
import frc.robot.Constants.PLACES;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.AutoSpeed;
import frc.robot.commands.ClimbSpeed;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootSpeed;
import frc.robot.commands.IndexerSpin;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.PIDIntake;
import frc.robot.commands.ZeroTurret;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANDle;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.PhotonSubsystem;
import frc.robot.subsystems.vision.Vision;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        private double MaxSpeed = OPERATOR_CONSTANTS.MAX_SPEED_LIMITER
                        * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                              // desired
                                                                              // top
        // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max
                                                                                          // angular velocity
        /* Setting up bindings for necessary control of the swerve drive platform */

        // used 2025 for side to side movement
        //
        private final SwerveRequest.RobotCentric rcDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1);
        @SuppressWarnings("unused")
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        @SuppressWarnings("unused")
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        // simulated objects
        public SwerveDriveSimulation driveSimulation = null;
        public IntakeSimulation.IntakeSide intakeSimulation = null;

        public Climb climb;
        public final Vision vision;
        public final Drive drive;
        private final Intake intake;
        private final Indexer indexer;
        private final Shooter shooter;
        private final Turret turret;
        @SuppressWarnings("unused")
        private final CANDle candle = new CANDle();

        private final CommandPS4Controller driveController = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.DRIVER_CONTROLER_PORT);
        private final CommandPS4Controller operatorController = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.OPERATOR_CONTROLER_PORT);
        private final CommandPS4Controller testController1 = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.TEST_CONTROLER1_PORT);
        private final CommandPS4Controller testController2 = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.TEST_CONTROLER2_PORT);

        private final SendableChooser<Command> autoChooser;
        private final SendableChooser<Command> bLineChouser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                climb = new Climb(new ClimbIOReal());
                                drive = new Drive(
                                                new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                                new ModuleIOTalonFX(TunerConstants.BackRight),
                                                (robotPose) -> {
                                                });
                                indexer = new Indexer(new IndexerIOReal());
                                intake = new Intake(new IntakeIOReal());
                                shooter = new Shooter(new ShooterIOReal());
                                vision = new Vision(drive::addVisionMeasurement);
                                turret = new Turret(new TurretIOReal());

                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                driveSimulation = new SwerveDriveSimulation(Drive.getMapleSimConfig(),
                                                new Pose2d(-3, -3, new Rotation2d()));
                                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                                drive = new Drive(
                                                new GyroIOSim(driveSimulation.getGyroSimulation()),
                                                new ModuleIOSim(driveSimulation.getModules()[0]),
                                                new ModuleIOSim(driveSimulation.getModules()[1]),
                                                new ModuleIOSim(driveSimulation.getModules()[2]),
                                                new ModuleIOSim(driveSimulation.getModules()[3]),
                                                driveSimulation::setSimulationWorldPose);

                                intake = new Intake(new IntakeIOSim(driveSimulation));
                                indexer = new Indexer(new IndexerIOSim());
                                climb = new Climb(new ClimbIOSim());
                                vision = new Vision(drive::addVisionMeasurement);
                                shooter = new Shooter(new ShooterIOSim());
                                turret = new Turret(new TurretIOSim());

                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                (robotPose) -> {
                                                });
                                intake = new Intake(new IntakeIO() {
                                });
                                indexer = new Indexer(new IndexerIO() {
                                });
                                climb = new Climb(new ClimbIO() {
                                });
                                vision = new Vision(drive::addVisionMeasurement);
                                shooter = new Shooter(new ShooterIO() {
                                });
                                turret = new Turret(new TurretIO() {
                                });

                                break;
                }

                // Configure the trigger bindings
                bLineChouser = new SendableChooser<Command>();
                configureBindings();
                // make commands for autos
                configureAutoCommands();
                // make bline autos
                buildBLineAutos();

                // For convenience a programmer could change this when going to competition.
                boolean isCompetition = false;
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer
                autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                                "testing",
                                (stream) -> isCompetition
                                                ? stream.filter(auto -> auto.getName().startsWith("comp"))
                                                : stream);
                bLineChouser.setDefaultOption("none", null);
                // SmartDashboard.putData("Auto Mode", autoChooser);

        }

        private void buildBLineAutos() {
                bLineChouser.addOption("test", new SequentialCommandGroup(drive.pathFromString("example_c")/*
                                                                                                            * ,
                                                                                                            * new
                                                                                                            * AimAtHub(
                                                                                                            * turret,
                                                                                                            * drive)
                                                                                                            */));

        }

        private void configureAutoCommands() {

                NamedCommands.registerCommand("aim'n'Shoot",
                                new AimAndShoot(shooter, turret, drive, PLACES.CENTER_OF_HUB));
                NamedCommands.registerCommand("shoot", new ShootSpeed(shooter, 20, false));
                NamedCommands.registerCommand("intake spin", new IntakeSpin(intake));
                // NamedCommands.registerCommand("indexer", new IndexerSpin(indexer,
                // IndexerStates.UP));
                NamedCommands.registerCommand("aim forward", new ManualTurret(turret, 0));
                NamedCommands.registerCommand("AimAtHub", new AimAtHub(turret, drive));
                NamedCommands.registerCommand("homeAll",
                                new SequentialCommandGroup(new ManualTurret(turret, 0),
                                                new PIDIntake(intake, IntakePosition.IN),
                                                new ShootSpeed(shooter, SHOOTER_CONSTANTS.SHOOTER_IDLE_SPEED,
                                                                false)/*
                                                                       * ,
                                                                       * new IndexerSpin(indexer, IndexerStates.OFF)
                                                                       */));
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {

                drive.setDefaultCommand(
                                DriveCommands.fieldOrientedDrive(
                                                drive,
                                                () -> -driveController.getLeftY(),
                                                () -> -driveController.getLeftX(),
                                                () -> -driveController.getRightX()));

                // speed up or slow down drivtrain command that overrides the default command
                driveController.R1()
                                .toggleOnTrue(DriveCommands.robotOrientedDrive(drive, () -> -driveController.getLeftY() * Constants.ROBOT_PROPERTIES.slowdownSpeed,
                                                () -> -driveController.getLeftX() * Constants.ROBOT_PROPERTIES.slowdownSpeed, () -> -driveController.getRightX()));
                // robot centric drive
                driveController.povUp()
                                .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 1, () -> 0, () -> 0));
                driveController.povDown()
                                .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> -1, () -> 0, () -> 0));
                driveController.povLeft()
                                .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 0, () -> 1, () -> 0));
                driveController.povRight()
                                .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 0, () -> -1, () -> 0));

                driveController.options().whileTrue(drive.followPathCommandtoTestPose());

                driveController.share()
                                .onTrue(DriveCommands.stopWithX(drive));

                driveController.R2().onTrue(drive.pathFindToHubShot());

                driveController.L1().onTrue(drive.commandResetOdometry(new Pose2d(3, 3, new Rotation2d())));



                final Runnable resetHeading = Constants.currentMode == Constants.Mode.SIM
                                ? () -> drive.resetOdometry(driveSimulation.getSimulatedDriveTrainPose())
                                : () -> drive.resetOdometry(
                                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
                driveController.triangle().onTrue(Commands.runOnce(resetHeading).ignoringDisable(true));

                // ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎

                // CANDle subsystem
                // operatorController.povUp()
                // .whileTrue(candle.setLEDSTate(CANDLE_STRIPS.FIRST,
                // LEDState.PINK));
                // DriverStation.getMatchTime();
                // DriverStation.getGameSpecificMessage().isEmpty()
                // operatorController.povLeft()
                // .whileTrue(candle.setLEDSTate(Constants.CANDLE_CONSTANTS.STRIPS.FIRST,
                // LEDState.COPPER));

                // operatorController.povRight().whileTrue(candle.fire(STRIPS.FIRST));

                // Turret subsystem
                operatorController.L1().whileTrue(new ManualTurret(turret, 0));
                operatorController.L2().whileTrue(new AimAndShoot(shooter, turret, drive, Constants.PLACES.CENTER_OF_HUB));

                operatorController.options().whileTrue(
                                turret.runEnd(() -> turret.setTurretSpeed(0.05), () -> turret.setTurretSpeed(0)));
                operatorController.share().whileTrue(
                                turret.runEnd(() -> turret.setTurretSpeed(-0.05), () -> turret.setTurretSpeed(0)));

                operatorController.touchpad().onTrue(new ZeroTurret(turret));

                // Shooter Subsystem
                operatorController.triangle().onTrue(new ShootSpeed(shooter, 0, false));

                operatorController.square().onTrue(new ShootSpeed(shooter, 17, false));

                operatorController.circle().whileTrue(new ClimbSpeed(climb, 0.25));

                operatorController.cross().whileTrue(new ClimbSpeed(climb, -0.25));

                // Indexer subsystem
                operatorController.R1().whileTrue(new IndexerSpin(indexer,
                                IndexerStates.UP));

                // Intake subsystem
                // out
                operatorController.povUp().whileTrue(new ManualIntake(intake, 0.075));
                // in
                operatorController.povDown().whileTrue(new ManualIntake(intake, -0.075));

                // roller
                operatorController.povLeft().whileTrue(new IntakeSpin(intake));

                // set aside for when the pid is tuned and constants are updated
                // operatorController.povRight().whileTrue(new PIDIntake(intake, null));

                // ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎

                // testController1.R1().whileTrue(new IndexerSpin(indexer,
                //                 IndexerStates.UP));

                // testController1.povDown().onTrue(new ShootSpeed(shooter, 0, false));
                // testController1.povUp().onTrue(new ShootSpeed(shooter, 20, false));
                // testController1.povRight().whileTrue(new AutoSpeed(shooter, drive));

                // testController1.triangle().onTrue(new ShootSpeed(shooter, 1, true));
                // testController1.square().onTrue(new ShootSpeed(shooter, -1, true));

                // testController1.circle().onTrue(new ShootSpeed(shooter, 5, true));
                // testController1.cross().onTrue(new ShootSpeed(shooter, -5, true));

                // testController2.povUp().onTrue(new ManualTurret(turret, 0));
                // testController2.povDown().onTrue(new ManualTurret(turret, 0.25));
                // testController2.povRight().whileTrue(new AimAtHub(turret, drive));

                // testController2.R1().whileTrue(new IndexerSpin(indexer,
                //                 IndexerStates.UP));
                // testController2.square().whileTrue(new AimAtHub(turret, drive));
                // testController2.triangle().whileTrue(new AutoSpeed(shooter, drive));
                // testController2.circle().toggleOnTrue(new AimAndShoot(shooter, turret, drive,
                //                 null));
                // testController2.cross().whileTrue(new ShootSpeed(shooter, 0, false));
        }

        public Command resetGyro() {
                return Commands.runOnce(() -> {
                        drive.resetOdometry(new Pose2d());
                });
        }

        public Command zeroThings() {
                return new ParallelCommandGroup(new ZeroTurret(turret), new ShootSpeed(shooter, 0, false));
        }

        /**
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // return autoChooser.getSelected();
                return bLineChouser.getSelected();
        }

        public void resetSimulation() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().resetFieldForAuto();
        }

}
