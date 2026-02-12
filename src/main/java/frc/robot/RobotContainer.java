// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.Constants.PLACES;
import frc.robot.Constants.CANDLE_STRIPS;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.PIDTurret;
import frc.robot.commands.Shoot;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.IndexerSpin;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.PIDIntake;
import frc.robot.commands.ZeroTurret;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANDle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.CANDle.LEDState;

import static edu.wpi.first.units.Units.*;

import java.time.Instant;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        // used 2025 for side to side movement
        // TODO: ask cole/evyln if they want robot centric drive this year?
        private final SwerveRequest.RobotCentric rcDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1);
        @SuppressWarnings("unused")
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        @SuppressWarnings("unused")
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        @SuppressWarnings("unused")
        private final Telemetry logger = new Telemetry(MaxSpeed);

        public final PhotonSubsystem vision = new PhotonSubsystem();
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final IntakeSubsystem intake = new IntakeSubsystem();
        private final IndexerSubsystem indexer = new IndexerSubsystem();
        private final ShooterSubsystem shooter = new ShooterSubsystem();
        private final TurretSubsystem turret = new TurretSubsystem();
        private final CANDle candle = new CANDle();

        private final CommandPS4Controller driveController = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.DRIVER_CONTROLER_PORT);
        private final CommandPS4Controller operatorController = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.OPERATOR_CONTROLER_PORT);
        private final CommandPS4Controller testController = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.TEST_CONTROLER_PORT);

        private final SendableChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // add telemetry
                drivetrain.registerTelemetry(logger::telemeterize);
                // Configure the trigger bindings
                configureBindings();
                // make commands for autos
                configureAutoCommands();

                // For convenience a programmer could change this when going to competition.
                boolean isCompetition = true;
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer
                autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                                (stream) -> isCompetition
                                                ? stream.filter(auto -> auto.getName().startsWith("comp"))
                                                : stream);
                SmartDashboard.putData("Auto Mode", autoChooser);

        }

        private void configureAutoCommands() {

                NamedCommands.registerCommand("aim'n'Shoot",
                                new AimAndShoot(shooter, turret, drivetrain, PLACES.CENTER_OF_HUB));

                // NamedCommands.registerCommand("Home", new SequentialCommandGroup(
                // new StopShooterWheel(shooter),
                // new RunCommand(() -> wristSubsystem
                // .setCurrentPosition(WristPosition.HOME),
                // wristSubsystem).withTimeout(0.1),
                // new PIDElevator(ElevatorPosition.Home, elevatorSubsystem),
                // new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0),
                // elevatorSubsystem).withTimeout(0.1)));

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

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive
                                                                                                       // forward
                                                                                                       // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left
                                                                                                       // with
                                                                                                       // negative X
                                                                                                       // (left)
                                                .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive
                                                                                                                   // counterclockwise
                                                                                                                   // with
                                // negative X (left)
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.

                // speed up or slow down drivtrain command that overrides the default command
                driveController.povRight().whileTrue(drivetrain
                                .applyRequest(() -> drive.withVelocityX(
                                                -1 * driveController.getLeftY() * MaxSpeed) // Drive
                                                                                            // forward
                                                                                            // with
                                                                                            // negative
                                                                                            // Y
                                                                                            // (forward)
                                                .withVelocityY(-1
                                                                * driveController.getLeftX()
                                                                * MaxSpeed) // Drive left
                                                                            // with negative
                                                                            // X (left)
                                                .withRotationalRate(-.7
                                                                * driveController.getRightX() * MaxAngularRate) // Drive
                                // counterclockwise
                                // with
                                // negative
                                // X
                                // (left)
                                ));

                driveController.R2().whileTrue(drivetrain.applyRequest(() -> rcDrive.withVelocityX(0.1 * MaxSpeed)));

                driveController.L2().whileTrue(drivetrain.applyRequest(() -> rcDrive.withVelocityX(-0.1 * MaxSpeed)));

                driveController.R1().whileTrue(drivetrain.applyRequest(() -> rcDrive.withVelocityY(-0.1 * MaxSpeed)));

                driveController.L1().whileTrue(drivetrain.applyRequest(() -> rcDrive.withVelocityY(0.1 * MaxSpeed)));

                driveController.triangle().onTrue(resetGyro());

                driveController.options().whileTrue(drivetrain.followPathCommandtoTestPose());

                // driveController.povDown().whileTrue(drivetrain.followPathCommandtoHUB());

                // operatorController.povUp().whileTrue(new AimAtHub(turret, drivetrain));

                // operatorController.share().onTrue(new ZeroTurret(turret));

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
                // operatorController.L1().whileTrue(new PIDTurret(turret, 0));

                // operatorController.L2().whileTrue(new PIDTurret(turret,
                // Utils.getAngleToHub(drivetrain)));

                // operatorController.cross().onTrue(new ZeroTurret(turret));

                // Shooter Subsystem
                operatorController.triangle().onTrue(new Shoot(shooter, 0));

                operatorController.square().onTrue(new Shoot(shooter, 50));

                operatorController.circle().onTrue(new Shoot(shooter, Utils.shooterSpeedFromDistance(
                                Utils.distanceFromPose(Constants.PLACES.CENTER_OF_HUB, drivetrain))));

                // Indexer subsystem
                operatorController.R1().whileTrue(new IndexerSpin(indexer));

                // Intake subsystem
                // out
                operatorController.povUp().whileTrue(new ManualIntake(intake, 0.075));
                // in
                operatorController.povDown().whileTrue(new ManualIntake(intake, -0.075));
                // roller
                operatorController.povLeft().whileTrue(new IntakeSpin(intake, -0.5));

                // set aside for when the pid is tuned and constants are updated
                // operatorController.povRight().whileTrue(new PIDIntake(intake, null));







                testController.povDown().onTrue(new Shoot(shooter, 0));
                testController.povLeft().onTrue(new Shoot(shooter, OPERATOR_CONSTANTS.setRPM));
                testController.povUp().onTrue(new Shoot(shooter, OPERATOR_CONSTANTS.setRPM+1));
                testController.povRight().onTrue(new Shoot(shooter, OPERATOR_CONSTANTS.setRPM+2));
                testController.square().onTrue(new Shoot(shooter, OPERATOR_CONSTANTS.setRPM+3));
                testController.triangle().onTrue(new Shoot(shooter, OPERATOR_CONSTANTS.setRPM+4));
                testController.circle().onTrue(new Shoot(shooter, OPERATOR_CONSTANTS.setRPM+5));
                testController.R2().whileTrue(new IndexerSpin(indexer));
        }

        public Command resetGyro() {
                return Commands.runOnce(() -> {
                        drivetrain.getPigeon2().reset();
                });
        }

        public Command zeroThings() {
                return new ParallelCommandGroup(new ZeroTurret(turret), new Shoot(shooter, 0));
        }

        // ZeroTurret(turret)
        /**
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}
