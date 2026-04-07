package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

public final class Autos {
        private static final SendableChooser<Command> chooser = new SendableChooser<>();

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
                return chooser.getSelected();
        }

        public static void putChouser() {
                chooser.setDefaultOption("Do Nothing", Commands.none());
                makeAutos();
                for (AutoDefinition autoDefinition : AUTOS) {
                        chooser.addOption(autoDefinition.name(), autoDefinition.command());
                }
                SmartDashboard.putData("AUTOS chouser", chooser);

        }

        private static Command outNSweepFirst() {
                return new SequentialCommandGroup(
                                drive.autoPathFromString("b1-1"),
                                new Intake_PID(intake, IntakePosition.OUT,
                                                OPERATOR_CONSTANTS.ROLLER_SPEED)
                                                .withTimeout(0.05),
                                drive.autoPathFromString("b1-2"),
                                new Intake_Spin(intake, 0),
                                drive.autoPathFromString("b1-3"));
        }
        

        private static void makeAutos() {
                AUTOS = List.of(
                                auto("8ball then out",
                                                new SequentialCommandGroup(
                                                                Shooter_AutoShoot_Sequence.get(shooter, turret, indexer)
                                                                                .withDeadline(new WaitCommand(2)),
                                                                outNSweepFirst())),
                                auto("out then out again",
                                                new SequentialCommandGroup(
                                                                outNSweepFirst(),
                                                                Shooter_AutoShoot_Sequence.get(shooter, turret,
                                                                                indexer).withTimeout(5),
                                                                outNSweepFirst(),
                                                                Shooter_AutoShoot_Sequence.get(shooter, turret,
                                                                                indexer))),
                                auto("autoclimb", Climb_AutoClimb_Sequence.get(drive, climb)),
                                auto("out",
                                                new SequentialCommandGroup(
                                                                outNSweepFirst(), Shooter_AutoShoot_Sequence.get(shooter, turret, indexer))),
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
