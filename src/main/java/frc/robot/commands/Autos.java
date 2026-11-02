package frc.robot.commands;

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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.driveUtils.BrutalClimbUtils;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.function.Supplier;

public final class Autos {
        // Uses Supplier<Command> to ensure a fresh, unpolluted command tree is constructed
        // every time autonomous runs, preventing WPILib multiple CommandGroup registration crashes.
        private static final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();

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
                Supplier<Command> selected = chooser.getSelected();
                return selected != null ? selected.get() : Commands.none();
        }

        public static void putChooser() {
                chooser.setDefaultOption("Do Nothing", Commands::none);
                makeAutos();
                for (AutoDefinition autoDefinition : AUTOS) {
                        chooser.addOption(autoDefinition.name(), autoDefinition.commandSupplier());
                }
                SmartDashboard.putData("AUTOS chooser", chooser);
        }

        private static Command outNSweepFirst() {
                return new SequentialCommandGroup(
                                new ParallelDeadlineGroup(drive.defer(() -> drive.autoPathFromString("b1-1")),
                                                new SequentialCommandGroup(new WaitCommand(1.0),
                                                                new Intake_PID(intake, IntakePosition.OUT,
                                                                                OPERATOR_CONSTANTS.ROLLER_SPEED))),
                                drive.defer(() -> drive.commandFromPath(drive.changeConstrains(
                                                drive.autoMirrorPath(drive.pathFromString("b1-2")),
                                                new PathConstraints().setMaxVelocityMetersPerSec(0.75))))
                                                .withTimeout(4.0),
                                new Intake_Spin(intake, 0, false),
                                drive.defer(() -> drive.autoPathFromString("b1-3")));
        }

        private static Command outNSweepBumpNum2() {
                return new ParallelDeadlineGroup(drive.defer(() -> drive.autoPathFromString("b1-4")),
                                new Intake_PID(intake, IntakePosition.OUT, OPERATOR_CONSTANTS.ROLLER_SPEED));
        }

        @SuppressWarnings("unused")
        private static Command toUpperClimb() {
                return new SequentialCommandGroup(
                                drive.defer(() -> drive.commandFromPath(drive.pathFromPose(FIELD_CONSTANTS.TEST_POSE2D))),
                                new ParallelCommandGroup(
                                                drive.defer(() -> drive.commandFromPath(BrutalClimbUtils.getPreClimbTarget(drive))),
                                                new Climb_Move(climb, ClimbPosition.UP)),
                                drive.defer(() -> drive.commandFromPath(BrutalClimbUtils.getFinalClimbTarget(drive))),
                                new Climb_Move(climb, ClimbPosition.CLIMBED));
        }

        private static void makeAutos() {
                AUTOS = List.of(
                                auto("8ball then out", () -> new SequentialCommandGroup(
                                                Shooter_AutoShoot_Sequence
                                                                .getWRumble(shooter, turret, indexer, intake)
                                                                .withDeadline(new WaitCommand(2.0)),
                                                outNSweepFirst())),
                                auto("out", () -> new SequentialCommandGroup(
                                                outNSweepFirst(),
                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                indexer, intake).withTimeout(3.0))),
                                auto("out then out again", () -> new SequentialCommandGroup(
                                                outNSweepFirst(),
                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                indexer, intake).withTimeout(9.0),
                                                outNSweepFirst(),
                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                indexer, intake))),
                                auto("out then back on bump", () -> new SequentialCommandGroup(
                                                outNSweepFirst(),
                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                indexer, intake).withTimeout(9.0),
                                                outNSweepBumpNum2(),
                                                Shooter_AutoShoot_Sequence.getWRumble(shooter, turret,
                                                                indexer, intake))),
                                auto("middle, back-shoot-depo-shoot-autoclimb", () -> new SequentialCommandGroup(
                                                drive.defer(() -> drive.commandFromPath(drive.pathFromString("m1-1"))),
                                                Shooter_AutoShoot_Sequence.get(shooter, turret, indexer)
                                                                .withTimeout(4.0),
                                                new ParallelDeadlineGroup(
                                                                drive.defer(() -> drive.commandFromPath(drive.pathFromString("m1-2"))),
                                                                new Intake_PID(intake, IntakePosition.OUT,
                                                                                OPERATOR_CONSTANTS.ROLLER_SPEED)),
                                                drive.defer(() -> drive.commandFromPath(drive.pathFromString("m1-3"))),
                                                new ParallelCommandGroup(
                                                                drive.defer(() -> drive.commandFromPath(drive.pathFromString("m1-4"))),
                                                                Shooter_AutoShoot_Sequence
                                                                                .getWRumble(shooter, turret, indexer,
                                                                                                intake)
                                                                                .withTimeout(5.5),
                                                                new Climb_Move(climb, ClimbPosition.UP)),
                                                drive.defer(() -> drive.commandFromPath(BrutalClimbUtils.getFinalClimbTarget(drive))),
                                                new Climb_Move(climb, ClimbPosition.CLIMBED))),
                                auto("drive forwards", () -> new SequentialCommandGroup(
                                                drive.cRunVelocity(new ChassisSpeeds(1, 0, 0))
                                                                .withTimeout(Seconds.of(1)),
                                                new Intake_PID(intake, IntakePosition.OUT, 0)))
                );
        }

        private static List<AutoDefinition> AUTOS = null;

        private static AutoDefinition auto(String name, Supplier<Command> commandSupplier) {
                return new AutoDefinition(name, commandSupplier);
        }

        public static Command waitSeconds(double seconds) {
                return new WaitCommand(seconds);
        }

        private record AutoDefinition(String name, Supplier<Command> commandSupplier) {
        }
}
