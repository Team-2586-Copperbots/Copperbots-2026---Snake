package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class Shooter_AutoShoot_Sequence {

    public static boolean shouldShoot() {
        return (Shooter.getInstance().isAtTarget() && Turret.getInstance().isAtTarget());
    }

    public static Command get(Shooter shooter, Turret turret, Indexer indexer) {
        return new ParallelCommandGroup(
                new Turret_AimAndShoot(shooter, turret),
                new Indexer_AutoFeed(indexer)
        // ,new Intake_Ratle(null)
        );
    }

    public static Command getWRumble(Shooter shooter, Turret turret, Indexer indexer, Intake intake) {
        return new ParallelCommandGroup(
                new Turret_AimAndShoot(shooter, turret),
                new Indexer_AutoFeed(indexer),
                new Intake_Ratle(intake, true));
    }
}
