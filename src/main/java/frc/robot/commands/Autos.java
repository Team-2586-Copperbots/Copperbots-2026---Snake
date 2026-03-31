package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.CANDle;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

public final class Autos {
    private final Climb climb = Climb.getInstance();
    private final Drive drive = Drive.getInstance();
    private final Vision vision = Vision.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Turret turret = Turret.getInstance();
    @SuppressWarnings("unused")
    private final CANDle candle = new CANDle();

    private Autos() {
    }

    public SendableChooser<Command> getChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", Commands.none());
        for (AutoDefinition autoDefinition : AUTOS) {
            chooser.addOption(autoDefinition.name(), autoDefinition.command());
        }

        return chooser;
    }

    private final List<AutoDefinition> AUTOS = List.of(
            auto("8ball", new SequentialCommandGroup(drive.pathFromString("b1-1"),
                    new Intake_PID(intake, IntakePosition.OUT,
                            0).withTimeout(0.05),
                    drive.pathFromString("b1-2"),
                    new Intake_Spin(intake, 0).withTimeout(0.05))),
            auto("drive forwards", new SequentialCommandGroup(drive
                    .cRunVelocity(new ChassisSpeeds(1, -1, 0)).withTimeout(Seconds.of(1)),
                    new Intake_PID(intake, IntakePosition.OUT, 0)))
    // auto(
    // "Simple Back",
    // new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()),
    // setSystem(DesiredSystemState.HOME),
    // setIntake(DesiredIntakeState.STOWED),
    // setClimber(DesiredClimbState.RETRACTED),
    // followPath("simple_back", true)
    // ),
    // auto(
    // "Citrus Sweep",
    // new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()),
    // setSystem(DesiredSystemState.HOME),
    // setIntake(DesiredIntakeState.DEPLOYED),
    // setClimber(DesiredClimbState.RETRACTED),
    // followPath("citrus_sweep"),
    // waitSeconds(0.25),
    // setIntake(DesiredIntakeState.STOWED)
    // )
    // auto(
    // "bottom_sweep_over",
    // new Pose2d(
    // new Translation2d(3.511, 2.160),
    // Rotation2d.fromRadians(Math.PI)),
    // setSystem(DesiredSystemState.HOME),
    // firstFollowPath("bottom_sweep_over", false, false)),
    // auto(
    // "top_sweep_short_depo",
    // new Pose2d(
    // new Translation2d(3.511, 2.160),
    // Rotation2d.fromRadians(Math.PI)),
    // new ParallelCommandGroup(
    // new SequentialCommandGroup(
    // setSystem(DesiredSystemState.HOME),
    // firstFollowPath("top_sweep_short_depo", false, false)),
    // new SequentialCommandGroup(
    // new WaitCommand(15.5),
    // setTarget(TargetState.HUB),
    // setSystem(DesiredSystemState.SHOOTING)))

    // ),
    // auto(
    // "straight",
    // new Pose2d(
    // new Translation2d(3.511, 2.160),
    // Rotation2d.fromRadians(Math.PI)),
    // setSystem(DesiredSystemState.HOME),
    // firstFollowPath("straight")),
    // auto(
    // "outpost",
    // new Pose2d(
    // new Translation2d(3.511, 2.160),
    // Rotation2d.fromRadians(Math.PI)),
    // setTarget(TargetState.HUB),
    // setSystem(DesiredSystemState.SHOOTING),
    // firstFollowPath("outpost")),
    // auto(
    // "overcharge",
    // new Pose2d(
    // new Translation2d(3.569, 2.320),
    // Rotation2d.fromRadians(0)),
    // setTarget(TargetState.HUB),
    // setSystem(DesiredSystemState.HOME),
    // firstFollowPath("bottom_jab_sharp"),
    // waitSeconds(5),
    // followPath("outpost"),
    // waitSeconds(10)),
    // auto(
    // "double_swipe_bottom",
    // new Pose2d(
    // new Translation2d(3.41, 2.27),
    // Rotation2d.fromDegrees(118)),
    // setTarget(TargetState.HUB),
    // setSystem(DesiredSystemState.HOME),
    // firstFollowPath("bottom_jab_sharp"),
    // waitSeconds(3.4),
    // setSystem(DesiredSystemState.HOME),
    // followPath("bottom_sweep_short"),
    // waitSeconds(6)),
    // auto(
    // "double_swipe_top",
    // new Pose2d(
    // new Translation2d(3.569, 5.68),
    // Rotation2d.fromRadians(0)),
    // setTarget(TargetState.HUB),
    // setSystem(DesiredSystemState.HOME),
    // firstFollowPath("bottom_jab_sharp", false, true),
    // waitSeconds(6),
    // followPath("bottom_sweep_short", false, true),
    // waitSeconds(6))

    );

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
