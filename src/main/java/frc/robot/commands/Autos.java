package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.lib.BLine.Path.PathConstraints;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.subsystems.drive.BLine_Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.driveUtils.ManualClimbUtils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

public final class Autos {
        private static final SendableChooser<Command> autoChouser = new SendableChooser<Command>();

        private static final SendableChooser<Double> delayChooser = new SendableChooser<Double>();

        private static final Climb climb = Climb.getInstance();
        private static final Drive drive = Drive.getInstance();
        @SuppressWarnings("unused")
        private static final Vision vision = Vision.getInstance();
        private static final Intake intake = Intake.getInstance();
        private static final Indexer indexer = Indexer.getInstance();
        private static final Shooter shooter = Shooter.getInstance();
        private static final Turret turret = Turret.getInstance();
        @SuppressWarnings("unused")
        private static final LED candle = LED.getInstance();

        private Autos() {
        }

        public static Command getAuto() {
                SequentialCommandGroup torun = new SequentialCommandGroup(Commands.none());

                if (delayChooser.getSelected() >= 2) {
                        torun.addCommands(new ParallelDeadlineGroup(new WaitCommand(delayChooser.getSelected()),
                                        Shooter_AutoShoot_Sequence.get(shooter, turret, indexer)));
                } else {
                        torun.addCommands(new WaitCommand(delayChooser.getSelected()));
                }

                Command auto = autoChouser.getSelected().asProxy();
                torun.addCommands(auto);

                return torun;
        }

        public static void putChouser() {
                autoChouser.setDefaultOption("Do Nothing", Commands.none());
                makeAutos();
                for (AutoDefinition autoDefinition : AUTOS) {
                        autoChouser.addOption(autoDefinition.name(), autoDefinition.command());
                }
                SmartDashboard.putData("AUTOS chouser", autoChouser);

                // delays
                delayChooser.setDefaultOption("0", 0.0);
                for (int i = 1; i < 20; i++) {
                        delayChooser.addOption("" + i, (double) i);
                }
                SmartDashboard.putData("Auto delay", delayChooser);

        }

        private static Command outNSweepFirst() {
                return new SequentialCommandGroup(
                                new ParallelDeadlineGroup(drive.defer(() -> drive.autoPathFromString("b1-1")),
                                                new SequentialCommandGroup(new WaitCommand(1),
                                                                new Intake_PID(intake, IntakePosition.OUT,
                                                                                OPERATOR_CONSTANTS.ROLLER_SPEED))),
                                // drive.autoPathFromString("b1-2"),
                                drive.defer(() -> drive.commandFromPath(drive.changeConstrains(
                                                drive.autoMirrorPath(drive.pathFromString("b1-2")),
                                                new PathConstraints().setMaxVelocityMetersPerSec(0.75))))
                                                .withTimeout(4),
                                new Intake_Spin(intake, 0, false),
                                drive.defer(() -> drive.autoPathFromString("b1-3")));
        }

        private static Command newOutNSweepFirst() {
                return new ParallelDeadlineGroup(
                                drive.defer(() -> drive.autoPathFromString("b1-1_3")),
                                new Intake_PID(intake, IntakePosition.OUT,
                                                OPERATOR_CONSTANTS.ROLLER_SPEED));
        }

        private static Command ountNSwepBumpNum2() {
                return new ParallelDeadlineGroup(drive.defer(() -> drive.autoPathFromString("b1-4")),
                                new Intake_PID(intake, IntakePosition.OUT, OPERATOR_CONSTANTS.ROLLER_SPEED));
        }

        @SuppressWarnings("unused")
        private static Command toUpperClimb() {
                return new SequentialCommandGroup(
                                drive.commandFromPath(drive.pathFromPose(FIELD_CONSTANTS.TEST_POSE2D)),
                                new ParallelCommandGroup(
                                                drive.commandFromPath(ManualClimbUtils.getPreClimbTarget(drive)),
                                                new Climb_move(climb, ClimbPosition.UP)),
                                drive.commandFromPath(ManualClimbUtils.getFinalClimbTarget(drive)),
                                new Climb_move(climb, ClimbPosition.CLIMBED));
        }

        private static void makeAutos() {
                AUTOS = List.of(auto("Do nothing", Commands.none()),
                                auto("shoot 8", Shooter_AutoShoot_Sequence.get(shooter, turret, indexer)),
                                auto("elims auto 1",
                                                new SequentialCommandGroup(
                                                                new ParallelDeadlineGroup(
                                                                                drive.defer(() -> drive
                                                                                                .autoPathFromString(
                                                                                                                "e-1")),
                                                                                new Intake_PID(intake,
                                                                                                IntakePosition.OUT,
                                                                                                OPERATOR_CONSTANTS.ROLLER_SPEED))
                                                                                .withTimeout(9.0),
                                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                                indexer,
                                                                                intake))),
                                auto("elims auto 2",
                                                new SequentialCommandGroup(
                                                                new ParallelDeadlineGroup(
                                                                                drive.defer(() -> drive
                                                                                                .autoPathFromString(
                                                                                                                "e-2 secret")),
                                                                                new Intake_PID(intake,
                                                                                                IntakePosition.OUT,
                                                                                                OPERATOR_CONSTANTS.ROLLER_SPEED))
                                                                                .withTimeout(5.0),
                                                                DriveCommands.stopWithX(drive))),
                                auto("out", new SequentialCommandGroup(
                                                outNSweepFirst(),
                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                indexer, intake))),
                                auto("new out then out again", new SequentialCommandGroup(
                                                newOutNSweepFirst(),
                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret, indexer,
                                                                intake).withTimeout(9),
                                                newOutNSweepFirst(),
                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret, indexer,
                                                                intake))),
                                auto("out then out again",
                                                new SequentialCommandGroup(
                                                                outNSweepFirst(),
                                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                                indexer, intake).withTimeout(9),
                                                                outNSweepFirst(),
                                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                                indexer, intake))),
                                auto("out then back on bump",
                                                new SequentialCommandGroup(
                                                                outNSweepFirst(),
                                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                                indexer, intake).withTimeout(9),
                                                                ountNSwepBumpNum2(),
                                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                                indexer, intake))),
                                auto("new out then back on bump",
                                                new SequentialCommandGroup(
                                                                newOutNSweepFirst(),
                                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                                indexer, intake).withTimeout(9),
                                                                ountNSwepBumpNum2(),
                                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                                indexer, intake))),
                                auto("middle, simple move and shoot",
                                                new SequentialCommandGroup(
                                                                drive.commandFromPath(drive.pathFromString("m2-1")),
                                                                Shooter_AutoShoot_Sequence.get(shooter, turret,
                                                                                indexer))),
                                auto("middle, depo and no kliemb", new SequentialCommandGroup(
                                                new ParallelDeadlineGroup(
                                                                drive.commandFromPath(drive.pathFromString("m1-2")),
                                                                new Intake_PID(intake, IntakePosition.OUT,
                                                                                OPERATOR_CONSTANTS.ROLLER_SPEED)),
                                                drive.commandFromPath(drive.pathFromString("m1-3")),
                                                Shooter_AutoShoot_Sequence
                                                                .getWRumble(shooter, turret, indexer,
                                                                                intake))),
                                auto("middle, depo and yay! kliemb", new SequentialCommandGroup(
                                                new ParallelDeadlineGroup(
                                                                drive.commandFromPath(drive.pathFromString("m1-2")),
                                                                new Intake_PID(intake, IntakePosition.OUT,
                                                                                OPERATOR_CONSTANTS.ROLLER_SPEED)),
                                                drive.commandFromPath(drive.pathFromString("m1-3")),
                                                new ParallelCommandGroup(
                                                                Shooter_AutoShoot_Sequence
                                                                                .getWRumble(shooter, turret, indexer,
                                                                                                intake),
                                                                new Climb_move(climb, ClimbPosition.UP)).withTimeout(5),
                                                new Intake_PID(intake, IntakePosition.IN, 0).withTimeout(1),
                                                drive.commandFromPath(drive.changeConstrains(
                                                                drive.pathFromString("m1-4"),
                                                                BLine_Constants.highTolerence
                                                                                .setMaxVelocityMetersPerSec(0.25))),
                                                // drive.commandFromPath(drive.pathFromPose(new Pose2d(0.98, 05.010,
                                                // new Rotation2d(Degrees.of(90))))),
                                                drive.cRunVelocity(new ChassisSpeeds(-0.25, 0, 0)).withTimeout(2),
                                                new Climb_move(climb, ClimbPosition.CLIMBED))),
                                auto("middle, sad kliemb", new SequentialCommandGroup(new ParallelCommandGroup(
                                                Shooter_AutoShoot_Sequence
                                                                .get(shooter, turret, indexer)
                                                                .withTimeout(5.5),
                                                new Climb_move(climb, ClimbPosition.UP)),
                                                new Intake_PID(intake, IntakePosition.IN, 0)
                                                                .withTimeout(1),
                                                drive.commandFromPath(drive.changeConstrains(
                                                                drive.pathFromString("m1-4"),
                                                                BLine_Constants.highTolerence
                                                                                .setMaxVelocityMetersPerSec(0.25))),
                                                // drive.commandFromPath(drive.pathFromPose(new Pose2d(0.98, 05.010,
                                                // new Rotation2d(Degrees.of(90))))),
                                                drive.cRunVelocity(new ChassisSpeeds(-0.25, 0, 0)).withTimeout(2),
                                                new Climb_move(climb, ClimbPosition.CLIMBED))),
                                auto("drive forwards",
                                                new SequentialCommandGroup(
                                                                drive.cRunVelocity(new ChassisSpeeds(1, 0, 0))
                                                                                .withTimeout(Seconds.of(1)),
                                                                new Intake_PID(intake, IntakePosition.OUT, 0)))

                );
        }

        private static List<AutoDefinition> AUTOS = null;

        private static AutoDefinition auto(String name, Command... commands) {
                SequentialCommandGroup auto = new SequentialCommandGroup(commands);
                auto.setName(name);
                return new AutoDefinition(name, auto);
        }

        public static Command waitSeconds(double seconds) {
                return new WaitCommand(seconds);
        }

        private record AutoDefinition(String name, Command command) {
        }

}
