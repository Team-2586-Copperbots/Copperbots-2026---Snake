// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.ClimbSpeed;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IndexerSpin;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.IntakePID;
import frc.robot.commands.IntakeRatle;
import frc.robot.commands.ShootSpeed;
import frc.robot.commands.ZeroTurret;
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
        private final CommandPS4Controller simControler = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.SIM_CONTROLER_PORT);

        @SuppressWarnings("unused")
        private final SendableChooser<Command> autoChooser;
        private final SendableChooser<Command> bLineChouser;
        private final SendableChooser<Command> characterizationChooser;

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
                bLineChouser.addOption("test", new SequentialCommandGroup(drive.pathFromString("example_c")/*
                                                                                                            * ,
                                                                                                            * new
                                                                                                            * AimAtHub(
                                                                                                            * turret,
                                                                                                            * drive)
                                                                                                            */));

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
                                new AimAndShoot(shooter, turret, drive));
                NamedCommands.registerCommand("shoot", new ShootSpeed(shooter, 20, false));
                NamedCommands.registerCommand("intake spin", new IntakeSpin(intake, 1));
                // NamedCommands.registerCommand("indexer", new IndexerSpin(indexer,
                // IndexerStates.UP));
                NamedCommands.registerCommand("aim forward", new ManualTurret(turret, 0));
                NamedCommands.registerCommand("AimAtHub", new AimAtHub(turret, drive));
                NamedCommands.registerCommand("homeAll",
                                new SequentialCommandGroup(new ManualTurret(turret, 0),
                                                new IntakePID(intake, IntakePosition.IN, 0),
                                                new ShootSpeed(shooter, OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED,
                                                                false)));

                FollowPath.registerEventTrigger("trigger?", new IntakePID(intake, IntakePosition.OUT, OPERATOR_CONSTANTS.ROLLER_SPEED));
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

                // // speed up or slow down drivtrain command that overrides the default command
                // driveController.R1()
                // .toggleOnTrue(DriveCommands.robotOrientedDrive(drive,
                // () -> -driveController.getLeftY()
                // * Constants.ROBOT_PROPERTIES.slowdownSpeed,
                // () -> -driveController.getLeftX()
                // * Constants.ROBOT_PROPERTIES.slowdownSpeed,
                // () -> -driveController.getRightX()));
                // robot centric drive
                // driveController.povUp()
                //                 .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 1, () -> 0,
                //                                 () -> -driveController.getRightX()));
                // driveController.povDown()
                //                 .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> -1, () -> 0,
                //                                 () -> -driveController.getRightX()));
                // driveController.povLeft()
                //                 .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 0, () -> 1, () -> 0));
                // driveController.povRight()
                //                 .whileTrue(DriveCommands.robotOrientedDrive(drive, () -> 0, () -> -1, () -> 0));

                

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
                                new AimAndShoot(shooter, turret, drive)/*
                                                                        * ,
                                                                        * new SequentialCommandGroup(new
                                                                        * WaitCommand(0.5),
                                                                        * new IndexerSpin(indexer, IndexerStates.UP))
                                                                        */));
                operatorController.touchpad().onTrue(new ParallelCommandGroup(
                                new IndexerSpin(indexer, IndexerStates.OFF),
                                new ShootSpeed(shooter, OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED, false),
                                new ManualTurret(turret, 0)));
                operatorController.circle().onTrue(new ShootSpeed(shooter, 30, false));

                // indexer sudsystem
                operatorController.square().onTrue(new IndexerSpin(indexer, IndexerStates.UP));
                operatorController.triangle().onTrue(new IndexerSpin(indexer, IndexerStates.OFF));

                // climb
                operatorController.R1().whileTrue(new ClimbSpeed(climb, 0.9));
                operatorController.R2().whileTrue(new ClimbSpeed(climb, -0.9));

                // pid intake
                operatorController.povUp().onTrue(new IntakePID(intake, IntakePosition.OUT, 0));
                operatorController.povDown()
                                .onTrue(new IntakePID(intake, IntakePosition.IN, OPERATOR_CONSTANTS.ROLLER_SPEED));
                operatorController.share().whileTrue(new IntakePID(intake, -0.1, OPERATOR_CONSTANTS.ROLLER_SPEED));
                operatorController.options().whileTrue(new IntakePID(intake, 0.1, OPERATOR_CONSTANTS.ROLLER_SPEED));

                // roller
                operatorController.povLeft()
                                .onTrue(new IntakeSpin(intake, OPERATOR_CONSTANTS.ROLLER_SPEED));
                operatorController.povRight().onTrue(new IntakeSpin(intake, 0));
                operatorController.cross().onTrue(new IntakeSpin(intake, -Constants.OPERATOR_CONSTANTS.ROLLER_SPEED));

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

                testController1.povLeft().onTrue(new ManualTurret(turret, .25));
                testController1.povRight().whileTrue(new AimAtHub(turret, drive));
                testController1.povUp().onTrue(new AimAndShoot(shooter, turret, drive));
                testController1.R1().whileTrue(new IndexerSpin(indexer, IndexerStates.UP));
        }

        public Command resetGyro() {
                return Commands.runOnce(() -> {
                        drive.resetOdometry(new Pose2d());
                });
        }

        public Command zeroThings() {
                return new ParallelCommandGroup(new ZeroTurret(turret), new ParallelCommandGroup(new ShootSpeed(shooter, 0, false),
                                new IndexerSpin(indexer, IndexerStates.OFF), new IntakeSpin(intake, 0)).withTimeout(5));
        }

        /**
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                boolean characterization = false;
                if (characterization) {
                        return characterizationChooser.getSelected();
                } else {
                        if (bLineChouser.getSelected() != null) {
                                return bLineChouser.getSelected();
                        } else {
                                return autoChooser.getSelected();
                        }
                }

        }

        public void resetSimulation() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                // driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().resetFieldForAuto();
        }

}
