// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.Mode;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.commands.Turret_AimAndShoot;
import frc.robot.commands.Autos;
import frc.robot.commands.Climb_AutoClimb_Sequence;
import frc.robot.commands.Climb_ZeroClimb;
import frc.robot.commands.Climb_Move;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Indexer_AutoSpeed;
import frc.robot.commands.Indexer_Spin;
import frc.robot.commands.Intake_Current_Ratle;
import frc.robot.commands.Intake_Spin;
import frc.robot.commands.Intake_Time_Ratle;
import frc.robot.commands.Shooter_AutoShoot_Sequence;
import frc.robot.commands.Turret_ManualTurret;
import frc.robot.commands.Intake_PID;
import frc.robot.commands.Shooter_ShootSpeed;
import frc.robot.commands.Turret_Aim;
import frc.robot.commands.Turret_ZeroTurret;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LED_Colour;
import frc.robot.subsystems.LED.LED_Strip;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.simsProjectile;
import frc.robot.util.driveUtils.ManualClimbUtils;

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

        private static boolean zeroed = false;

        // MARK: Objects

        private final Climb climb = Climb.getInstance();
        private final Drive drive = Drive.getInstance();
        @SuppressWarnings("unused")
        private final Vision vision = Vision.getInstance();
        private final Intake intake = Intake.getInstance();
        private final Indexer indexer = Indexer.getInstance();
        private final Shooter shooter = Shooter.getInstance();
        private final Turret turret = Turret.getInstance();
        private final LED led = LED.getInstance();

        private final CommandPS4Controller driveController = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.DRIVER_CONTROLER_PORT);
        private final CommandPS4Controller operatorController = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.OPERATOR_CONTROLER_PORT);
        private final CommandPS4Controller testController1 = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.TEST_CONTROLER1_PORT);
        @SuppressWarnings("unused")
        private final CommandPS4Controller simControler = new CommandPS4Controller(
                        OPERATOR_CONSTANTS.SIM_CONTROLER_PORT);

        // private final SendableChooser<Command> autoChooser;
        private final SendableChooser<Command> bLineChouser;
        private final SendableChooser<Command> characterizationChooser;
        private final SendableChooser<Double> polarityChooser;
        public static final SendableChooser<Boolean> autofliper = new SendableChooser<Boolean>();

        /**
         * MARK: Init
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                if (Constants.currentMode == Mode.SIM) {
                        simsProjectile.createSimsProjectile(drive::getPose, () -> ChassisSpeeds
                                        .fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()),
                                        turret::getRotation, shooter::getMotor1Speed);
                        @SuppressWarnings("unused")
                        IntakeSimulation.IntakeSide intakeSimulation = null;

                }

                Autos.putChouser();

                autofliper.addOption("left", true);
                autofliper.setDefaultOption("right", false);
                SmartDashboard.putData("autofliper, default left", autofliper);
                polarityChooser = new SendableChooser<Double>();
                polarityChooser.addOption("negative", -1.0);
                polarityChooser.setDefaultOption("pos", 1.0);
                SmartDashboard.putData("Polarity chooser", polarityChooser);

                // Configure the trigger bindings
                bLineChouser = new SendableChooser<Command>();
                // make commands for autos
                configureAutoCommands();
                // make bline autos
                buildBLineAutos();
                // make chouser for drive charecterization
                characterizationChooser = new SendableChooser<Command>();
                addOptionsForCharecterization();

                // For convenience a programmer could change this when going to competition.
                // boolean isCompetition = false;
                // Build an auto chooser. This will use Commands.none() as the default option.
                // As an example, this will only show autos that start with "comp" while at
                // competition as defined by the programmer
                // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                // (stream) -> isCompetition
                // ? stream.filter(auto -> auto.getName().startsWith("comp"))
                // : stream);
                // SmartDashboard.putData("pathplaner chooser", autoChooser);
                SmartDashboard.putData("bline chooser", bLineChouser);
                SmartDashboard.putData("characterization Chooset", characterizationChooser);
                configureBindings();

        }

        private void buildBLineAutos() {
                // MARK: BLine
                bLineChouser.setDefaultOption("8ball", new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new Turret_AimAndShoot(shooter, turret),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(1),
                                                                new Indexer_Spin(indexer, IndexerStates.ON)))));

        }

        private void addOptionsForCharecterization() {
                // MARK: Drive Sysid
                // Set up SysId routines
                characterizationChooser.setDefaultOption("Not in use", Commands.none());
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
                // MARK: BLine commands
                NamedCommands.registerCommand("aim'n'Shoot",
                                new Turret_AimAndShoot(shooter, turret));
                NamedCommands.registerCommand("shoot", new Shooter_ShootSpeed(shooter, 20, false));
                NamedCommands.registerCommand("intake spin", new Intake_Spin(intake, 1, false));
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

                // MARK: Driver
                drive.setDefaultCommand(DriveCommands.myDrive(drive, driveController,
                                OPERATOR_CONSTANTS.MAX_SPEED_LIMITER, polarityChooser::getSelected));

                drive.resetOdometry(new Pose2d(2, 2, new Rotation2d()));

                // speed up or slow down drivtrain command that overrides the default command
                // driveController.cross().whileTrue(DriveCommands.myDrive(drive,
                // driveController,
                // OPERATOR_CONSTANTS.SLOW_SPEED_LIMITER, polarityChooser::getSelected));

                driveController.R2().whileTrue(new ParallelCommandGroup(
                                Shooter_AutoShoot_Sequence.get(shooter, turret, indexer),
                                DriveCommands.myDrive(drive, driveController, .4, polarityChooser::getSelected)));
                driveController.R1().toggleOnTrue(new Turret_Aim(turret));
                driveController.L2().onTrue(DriveCommands.stopWithX(drive));

                // driveController.povUp().whileTrue(drive
                // .sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));

                // driveController.circle().whileTrue(drive.deferedCommandToPose(FIELD_CONSTANTS.TEST_POSE2D));
                // driveController.triangle()
                // .whileTrue(drive.commandFromPath(ManualClimbUtils.getFinalClimbTarget(drive)));
                // driveController.square().whileTrue(Autos.getAuto());
                // driveController.cross().whileTrue(Climb_AutoClimb_Sequence.getManual(drive,
                // climb));
                driveController.L1().whileTrue(new Indexer_Spin(indexer, IndexerStates.ON));

                // MARK: Operator

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

                operatorController.square()
                                .onTrue(new Shooter_ShootSpeed(shooter, 0, false));

                // indexer sudsystem
                operatorController.circle().whileTrue(new Indexer_Spin(indexer, IndexerStates.ON));
                operatorController.triangle().onTrue(new Indexer_Spin(indexer, IndexerStates.OFF));
                operatorController.touchpad().onTrue(led.setColor(LED_Strip.FIRST, LED_Colour.BLACK));
                operatorController.PS().onTrue(led.setColor(LED_Strip.FIRST, LED_Colour.BLUE));

                // climb
                operatorController.R1().whileTrue(new Climb_Move(climb, 0.9));
                operatorController.R2().whileTrue(new Climb_Move(climb, -0.9));
                operatorController.L1().onTrue(new Climb_ZeroClimb(climb));

                // pid intake
                operatorController.povUp()
                                .onTrue(new Intake_PID(intake, IntakePosition.OUT, OPERATOR_CONSTANTS.ROLLER_SPEED));
                operatorController.povDown()
                                .onTrue(new Intake_PID(intake, IntakePosition.IN, OPERATOR_CONSTANTS.ROLLER_SPEED));
                operatorController.share().whileTrue(new Intake_PID(intake, 0.2, 0));
                operatorController.options().whileTrue(new Intake_PID(intake, -0.2, 0));

                // roller
                operatorController.povLeft()
                                .onTrue(new Intake_Spin(intake, OPERATOR_CONSTANTS.ROLLER_SPEED, false));
                operatorController.povRight().onTrue(new Intake_Spin(intake, 0, false));
                operatorController.cross()
                                .whileTrue(new Intake_Spin(intake, -Constants.OPERATOR_CONSTANTS.ROLLER_SPEED, true));

                // MARK: Test1

                // // code to test the drive
                // drive.setDefaultCommand(DriveCommands.myDrive(drive, testController1,
                //                 OPERATOR_CONSTANTS.MAX_SPEED_LIMITER, polarityChooser::getSelected));

                // // code to test the shooter/turret
                // testController1.R2().whileTrue(new ParallelCommandGroup(
                //                 new Turret_AimAndShoot(shooter, turret),
                //                 DriveCommands.myDrive(drive, testController1, .4, polarityChooser::getSelected)));
                // testController1.R1().toggleOnTrue(new Turret_Aim(turret));

                // code to test the intak
                // testController1.povUp().onTrue(new Intake_PID(intake, IntakePosition.OUT, 0));
                // testController1.povDown().onTrue(new Intake_PID(intake, IntakePosition.IN, 0));
                // testController1.povRight().whileTrue(new Intake_Spin(intake, 0, false));
                // testController1.povLeft().whileTrue(new Intake_Spin(intake, OPERATOR_CONSTANTS.ROLLER_SPEED, false));
                // testController1.triangle().whileTrue(new Intake_Time_Ratle(intake));
                // testController1.cross().whileTrue(new Intake_Current_Ratle(intake));

                // // code to test the indexer
                // testController1.povUp().whileTrue(new Indexer_AutoSpeed(indexer));
                // testController1.L1().whileTrue(new Indexer_Spin(indexer, IndexerStates.ON));
                // testController1.L2().whileTrue(new Indexer_Spin(indexer, new IndexerState(-0.45, 0.5)));
                // testController1.povRight().whileTrue(new Indexer_Spin(indexer,
                // IndexerStates.ON));
                // testController1.povLeft().onTrue(new Indexer_Spin(indexer,
                // IndexerStates.OFF));

                // // code to test the climb/autoclimb
                // testController1.circle().onTrue(new Climb_move(climb, ClimbPosition.UP));
                // testController1.square().onTrue(new Climb_move(climb, ClimbPosition.DOWN));
                // testController1.triangle().whileTrue(new Climb_move(climb, 1));
                // testController1.cross().whileTrue(new Climb_move(climb, -0.6));
                // testController1.options().onTrue(new Climb_ZeroClimb(climb));
                // testController1.povUp()
                // .whileTrue(drive.commandFromPath(drive.pathFromPoseWithConstraints(
                // new Pose2d(MathedClimbUtils.centerOfClimbPose, Rotation2d.kCCW_90deg),
                // BLine_Constants.highTolerence)));
                // testController1.L2().whileTrue(DriveCommands.myDrive(drive, testController1,
                // 1.0, polarityChooser::getSelected));
                // testController1.triangle().onTrue(new Climb_ZeroClimb(climb));
                // testController1.square().whileTrue(Climb_AutoClimb_Sequence.get(drive,
                // climb));
                // testController1.circle().onTrue(new Climb_Move(climb, ClimbPosition.UP));

                // // code for getting speeds
                testController1.triangle().onTrue(new Shooter_ShootSpeed(shooter, 55, false));
                testController1.cross().onTrue(new Shooter_ShootSpeed(shooter, 0, false));
                // testController1.triangle().onTrue(new Shooter_ShootSpeed(shooter, 5, true));
                // testController1.square().onTrue(new Shooter_ShootSpeed(shooter, -5, true));
                // testController1.circle().onTrue(new Shooter_ShootSpeed(shooter, 1, true));
                // testController1.cross().onTrue(new Shooter_ShootSpeed(shooter, -1, true));
        }

        public Command resetGyro() {
                return Commands.runOnce(() -> {
                        drive.resetOdometry(new Pose2d());
                });
        }

        public Command zeroThings() {
                if (!zeroed) {
                        zeroed = true;
                        return new ParallelCommandGroup(
                                        new Turret_ZeroTurret(turret),
                                        new Climb_ZeroClimb(climb),
                                        new Shooter_ShootSpeed(shooter, 0, false),
                                        new Indexer_Spin(indexer, IndexerStates.OFF).withTimeout(0.04),
                                        new Intake_Spin(intake, 0, false),
                                        DriveCommands.stopWithX(drive));

                }
                return Commands.none();
        }

        /**
         * MARK: Autonomous
         * 
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // return characterizationChooser.getSelected();
                return Autos.getAuto();

        }

        public void resetSimulation() {
                if (Constants.currentMode != Constants.Mode.SIM)
                        return;

                // driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().resetFieldForAuto();
        }

}
