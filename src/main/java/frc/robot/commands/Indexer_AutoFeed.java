package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class Indexer_AutoFeed extends Command {
    // Command that runs the spindexer and feed tower when the shooter and turret are lined up
    // Includes a 350ms feed-hold debounce to prevent start-stop stuttering when the flywheel speed dips as balls enter
    private static final double FEED_HOLD_TIME_SECS = 0.35; //TODO this may be a little long

    private final Indexer indexer;
    private final Shooter shooter;
    private final Turret turret;

    private double lastReadyTimestamp = -1.0;

    public Indexer_AutoFeed(Indexer indexerSubsystem) {
        this.indexer = indexerSubsystem;
        this.shooter = Shooter.getInstance();
        this.turret = Turret.getInstance();
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        lastReadyTimestamp = -1.0;
    }

    public boolean isTargetReady() {
        return shooter.isAtTarget() && turret.isAtTarget();
    }

    public boolean shouldShoot() {
        double now = Timer.getFPGATimestamp();
        return (lastReadyTimestamp > 0 && (now - lastReadyTimestamp) < FEED_HOLD_TIME_SECS);
    }

    @Override
    public void execute() {
        boolean targetReady = isTargetReady();
        double now = Timer.getFPGATimestamp();

        if (targetReady) {
            lastReadyTimestamp = now;
        }

        boolean feeding = shouldShoot();

        Logger.recordOutput("AutoFeed/Shooter.isAtTarget()", shooter.isAtTarget());
        Logger.recordOutput("AutoFeed/Turret.isAtTarget", turret.isAtTarget());
        Logger.recordOutput("AutoFeed/TargetReady", targetReady);
        Logger.recordOutput("AutoFeed/Feeding", feeding);

        if (feeding) {
            indexer.setSpindexerSpeed(IndexerStates.ON.spindexer);
            indexer.setTowerSpeed(IndexerStates.ON.tower);
        } else {
            indexer.setSpindexerSpeed(IndexerStates.OFF.spindexer);
            indexer.setTowerSpeed(IndexerStates.OFF.tower);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        lastReadyTimestamp = -1.0;
        indexer.setSpindexerSpeed(IndexerStates.OFF.spindexer);
        indexer.setTowerSpeed(IndexerStates.OFF.tower);
    }
}
