// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.commands.Turret_AimAndShoot;
import frc.robot.commands.Turret_AimAtHub;
import frc.robot.commands.Climb_move;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Indexer_Spin;
import frc.robot.commands.Intake_Spin;
import frc.robot.commands.Turret_ManualTurret;
import frc.robot.commands.Intake_PID;
import frc.robot.commands.Shooter_ShootSpeed;
import frc.robot.commands.Turret_ZeroTurret;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.CANDle;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeIO;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.GeneralUtils;
import frc.robot.util.simsProjectile;

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
        // private double MaxSpeed = OPERATOR_CONSTANTS.MAX_SPEED_LIMITER
        // * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
        // // desired
        // // top
        // // speed
        // private double MaxAngularRate =
        // RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
        // // second
        // // max
        // // angular velocity

        // simulated objects
        public SwerveDriveSimulation driveSimulation = null;
        public IntakeSimulation.IntakeSide intakeSimulation = null;

        public Climb climb;
        // public final PhotonSubsystem photonSubsystem;
        public final Drive drive;
        public final Vision vision;
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
        @SuppressWarnings("unused")
        private final CommandPS4Controller simControler = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.SIM_CONTROLER_PORT);

        private final SendableChooser<Command> autoChooser;
        private final SendableChooser<Command> bLineChouser;
        private final SendableChooser<Command> characterizationChooser;

        private final SendableChooser<Double> polarityChooser;

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
                                // photonSubsystem = new PhotonSubsystem();
                                vision = new Vision(drive::addVisionMeasurement, new VisionIOPhotonVision(
                                                VisionConstants.backCamera, VisionConstants.robotToBackCamera));

                                turret = new Turret(new TurretIOReal());

                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                driveSimulation = new SwerveDriveSimulation(Drive.getMapleSimConfig(),
                                                new Pose2d(2, 2, new Rotation2d()));
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
                                // photonSubsystem = new PhotonSubsystem();
                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOPhotonVisionSim(VisionConstants.backCamera,
                                                                VisionConstants.robotToBackCamera,
                                                                driveSimulation::getSimulatedDriveTrainPose));
                                shooter = new Shooter(new ShooterIOSim());
                                turret = new Turret(new TurretIOSim());

                                simsProjectile.createSimsProjectile(driveSimulation::getSimulatedDriveTrainPose,
                                                driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                                                turret::getRobotRelitiveRotation2D, shooter::getMotor1Speed);

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
                                // photonSubsystem = new PhotonSubsystem();
                                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                                });
                                shooter = new Shooter(new ShooterIO() {
                                });
                                turret = new Turret(new TurretIO() {
                                });

                                break;

                }

                polarityChooser = new SendableChooser<Double>();
                polarityChooser.addOption("negative", -1.0);
                polarityChooser.setDefaultOption("pos", 1.0);
                SmartDashboard.putData("Polarity chooser", polarityChooser);

                // Configure the trigger bindings
                bLineChouser = new SendableChooser<Command>();
                configureBindings();
                // make commands for autos
                configureAutoCommands();
                // make bline autos
                buildBLineAutos();
                // make chouser for drive charecterization
                characterizationChooser = new SendableChooser<Command>();
                addOptionsForCharecterization();

                // For convenience a programmer could change this when going to competition.
                boolean isCompetition = false;
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer
                autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                                (stream) -> isCompetition
                                                ? stream.filter(auto -> auto.getName().startsWith("comp"))
                                                : stream);
                SmartDashboard.putData("pathplaner chooser", autoChooser);
                SmartDashboard.putData("bline chooser", bLineChouser);
                SmartDashboard.putData("characterization Chooset", characterizationChooser);

        }

        private void buildBLineAutos() {
                bLineChouser.addOption("test", drive.pathFromString("pathv"));
                // bLineChouser.addOption("8ball", new
                // SequentialCommandGroup(drive.pathFromString("pathc"),
                // new ParallelCommandGroup(new AimAndShoot(shooter, turret, drive),
                // new SequentialCommandGroup(new WaitCommand(.2),
                // new IndexerSpin(indexer, IndexerStates.UP)))));
                bLineChouser.setDefaultOption("8ball", new SequentialCommandGroup(drive.resetHearding(),
                                new ParallelCommandGroup(new Turret_AimAndShoot(shooter, turret, drive),
                                                new SequentialCommandGroup(new WaitCommand(.2),
                                                                new Indexer_Spin(indexer, IndexerStates.ON)))));
                bLineChouser.addOption("shoot",
                                new Shooter_ShootSpeed(shooter, GeneralUtils.shooterSpeedFromDistance(
                                                GeneralUtils.distanceFromPose(FIELD_CONSTANTS.CENTER_OF_HUB, drive)),
                                                false));
                bLineChouser.addOption("8ball then out",
                                new SequentialCommandGroup(new ParallelDeadlineGroup(new WaitCommand(10),
                                                new ParallelCommandGroup(new Turret_AimAndShoot(shooter, turret, drive),
                                                                new SequentialCommandGroup(new WaitCommand(.5),
                                                                                new Indexer_Spin(indexer,
                                                                                                IndexerStates.ON)))),
                                                drive.pathFromString("b1-1"),
                                                new Intake_PID(intake, IntakePosition.OUT,
                                                                OPERATOR_CONSTANTS.ROLLER_SPEED).withTimeout(0.05),
                                                drive.pathFromString("b1-2"),
                                                new Intake_Spin(intake, 0).withTimeout(0.05),
                                                drive.pathFromString("b1-3"),
                                                new ParallelCommandGroup(new Turret_AimAndShoot(shooter, turret, drive),
                                                                new SequentialCommandGroup(new WaitCommand(0.5),
                                                                                new Indexer_Spin(indexer,
                                                                                                IndexerStates.ON)))));
                bLineChouser.addOption("drive forwards", new SequentialCommandGroup(drive
                                .cRunVelocity(new ChassisSpeeds(1, 0, 0)).withTimeout(Time.ofBaseUnits(0.5, Seconds)),
                                new Intake_PID(intake, IntakePosition.OUT, 0)));

        }

        private void addOptionsForCharecterization() {
                // Set up SysId routines
                characterizationChooser.addOption("Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                characterizationChooser.addOption("Drive Simple FF Characterization",
                                DriveCommands.feedforwardCharacterization(drive));
                characterizationChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                characterizationChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                characterizationChooser.addOption("Drive SysId (Dynamic Forward)",
                                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                characterizationChooser.addOption("Drive SysId (Dynamic Reverse)",
                                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        }

        private void configureAutoCommands() {

                NamedCommands.registerCommand("aim'n'Shoot",
                                new Turret_AimAndShoot(shooter, turret, drive));
                NamedCommands.registerCommand("shoot", new Shooter_ShootSpeed(shooter, 20, false));
                NamedCommands.registerCommand("intake spin", new Intake_Spin(intake, 1));
                NamedCommands.registerCommand("intake out",
                                new Intake_PID(intake, IntakePosition.OUT, OPERATOR_CONSTANTS.ROLLER_SPEED));
                NamedCommands.registerCommand("intake in",
                                new Intake_PID(intake, IntakePosition.IN, OPERATOR_CONSTANTS.ROLLER_SPEED));
                NamedCommands.registerCommand("indexer on", new Indexer_Spin(indexer, IndexerStates.ON));
                NamedCommands.registerCommand("indexer off", new Indexer_Spin(indexer, IndexerStates.OFF));
                NamedCommands.registerCommand("homeAll",
                                new SequentialCommandGroup(new Turret_ManualTurret(turret, 0),
                                                new Intake_PID(intake, IntakePosition.IN, 0),
                                                new Shooter_ShootSpeed(shooter, OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED,
                                                                false)));

                FollowPath.registerEventTrigger("intake out",
                                new Intake_PID(intake, IntakePosition.OUT, OPERATOR_CONSTANTS.ROLLER_SPEED)
                                                .withTimeout(0.5));
                FollowPath.registerEventTrigger("intake in",
                                new Intake_PID(intake, IntakePosition.IN, OPERATOR_CONSTANTS.ROLLER_SPEED));
                FollowPath.registerEventTrigger("aim n shoot",
                                new ParallelCommandGroup(new Turret_AimAndShoot(shooter, turret, drive),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(Time.ofBaseUnits(.5, Seconds)),
                                                                new Indexer_Spin(indexer, IndexerStates.ON))));
                FollowPath.registerEventTrigger("stop shooter", new ParallelCommandGroup(
                                new Shooter_ShootSpeed(shooter, OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED, false),
                                new Indexer_Spin(indexer, IndexerStates.OFF)));
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
                                                () -> -GeneralUtils.squareNumber(driveController.getLeftY()
                                                                * polarityChooser.getSelected()),
                                                () -> -GeneralUtils.squareNumber(driveController.getLeftX()
                                                                * polarityChooser.getSelected()),
                                                () -> -driveController.getRightX()));

                driveController.triangle().onTrue(drive.resetHearding());

                // speed up or slow down drivtrain command that overrides the default command
                driveController.cross()
                                .whileTrue(DriveCommands.fieldOrientedDrive(drive,
                                                () -> -GeneralUtils.squareNumber(driveController.getLeftY())
                                                                * OPERATOR_CONSTANTS.SLOW_SPEED_LIMITER,
                                                () -> -GeneralUtils.squareNumber(driveController.getLeftX())
                                                                * OPERATOR_CONSTANTS.SLOW_SPEED_LIMITER,
                                                () -> -driveController.getRightX()));
                // robot centric drive
                // driveController.povUp()
                // .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 1, () -> 0,
                // () -> -driveController.getRightX()));
                // driveController.povDown()
                // .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> -1, () -> 0,
                // () -> -driveController.getRightX()));
                // driveController.povLeft()
                // .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 0, () -> 1, () ->
                // 0));
                // driveController.povRight()
                // .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 0, () -> -1, () ->
                // 0));

                driveController.povUp().whileTrue(drive
                                .sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));

                driveController.circle().onTrue(drive.commandResetOdometry(new Pose2d(2, 2, new Rotation2d())));

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

                // Shooter + turret Subsystems
                operatorController.L2().onTrue(new ParallelCommandGroup(
                                new Turret_AimAndShoot(shooter, turret, drive)/*
                                                                               * ,
                                                                               * new SequentialCommandGroup(new
                                                                               * WaitCommand(0.5),
                                                                               * new IndexerSpin(indexer,
                                                                               * IndexerStates.UP))
                                                                               */));
                operatorController.touchpad().onTrue(new ParallelCommandGroup(
                                new Indexer_Spin(indexer, IndexerStates.OFF),
                                new Shooter_ShootSpeed(shooter, OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED, false),
                                new Turret_ManualTurret(turret, 0)));
                operatorController.square()
                                .onTrue(new Shooter_ShootSpeed(shooter, OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED, false));

                // indexer sudsystem
                operatorController.circle().whileTrue(new Indexer_Spin(indexer, IndexerStates.ON));
                operatorController.triangle().onTrue(new Indexer_Spin(indexer, IndexerStates.OFF));

                // climb
                operatorController.R1().whileTrue(new Climb_move(climb, 0.9));
                operatorController.R2().whileTrue(new Climb_move(climb, -0.9));

                // pid intake
                operatorController.povUp()
                                .onTrue(new Intake_PID(intake, IntakePosition.OUT, OPERATOR_CONSTANTS.ROLLER_SPEED));
                operatorController.povDown()
                                .onTrue(new Intake_PID(intake, IntakePosition.IN, OPERATOR_CONSTANTS.ROLLER_SPEED));
                operatorController.share().whileTrue(new Intake_PID(intake, 0.2, 0));
                operatorController.options().whileTrue(new Intake_PID(intake, -0.2, 0));

                // roller
                operatorController.povLeft()
                                .onTrue(new Intake_Spin(intake, OPERATOR_CONSTANTS.ROLLER_SPEED));
                operatorController.povRight().onTrue(new Intake_Spin(intake, 0));
                operatorController.cross().onTrue(new Intake_Spin(intake, -Constants.OPERATOR_CONSTANTS.ROLLER_SPEED));

                // ∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎∎

                // testController1.R1().whileTrue(new IndexerSpin(indexer,
                // IndexerStates.UP));

                // testController1.circle().whileTrue(new ShootSpeed(shooter, 30, false));
                // testController1.povLeft()
                // .onTrue(new IntakeSpin(intake, Constants.OPERATOR_CONSTANTS.ROLLER_SPEED));
                // testController1.povRight().onTrue(new IntakeSpin(intake, 0));

                // testController1.options().whileTrue(new IntakePID(intake, 0.1,
                // OPERATOR_CONSTANTS.ROLLER_SPEED));
                // testController1.share().whileTrue(new IntakePID(intake, -0.1,
                // OPERATOR_CONSTANTS.ROLLER_SPEED));

                // // pid intake
                // testController1.povUp().onTrue(new IntakePID(intake, IntakePosition.OUT, 0));
                // testController1.povDown().onTrue(new IntakePID(intake,
                // IntakePosition.HALFWAY, 0));
                // testController1.touchpad().whileTrue(new IntakeRatle(intake));

                testController1.povLeft().onTrue(new Turret_ManualTurret(turret, .25));
                testController1.povRight().whileTrue(new Turret_AimAtHub(turret, drive));
                testController1.povUp().onTrue(new Turret_AimAndShoot(shooter, turret, drive));
                testController1.R1().whileTrue(new Indexer_Spin(indexer, IndexerStates.ON));
        }

        public Command resetGyro() {
                return Commands.runOnce(() -> {
                        drive.resetOdometry(new Pose2d());
                });
        }

        public Command zeroThings() {
                return new ParallelCommandGroup(new Turret_ZeroTurret(turret),
                                new ParallelCommandGroup(new Shooter_ShootSpeed(shooter, 0, false),
                                                new Indexer_Spin(indexer, IndexerStates.OFF),
                                                new Intake_Spin(intake, 0))
                                                .withTimeout(1));
        }

        /**
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // boolean characterization = false;
                // if (characterization) {
                // return characterizationChooser.getSelected();
                // } else {
                // if (bLineChouser.getSelected() != null) {
                return bLineChouser.getSelected();
                // } else {
                // return autoChooser.getSelected();
                // }
                // }

        }

        public void resetSimulation() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                // driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().resetFieldForAuto();
        }

}
