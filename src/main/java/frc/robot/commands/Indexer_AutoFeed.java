package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public class Indexer_AutoFeed extends Command {
    private Indexer indexer;
    private Shooter shooter;
    private Turret turret;

    public Indexer_AutoFeed(Indexer indexerSubsystem) {
        this.indexer = indexerSubsystem;
        this.shooter = Shooter.getInstance();
        this.turret = Turret.getInstance();
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Logger.recordOutput("AutoFeed/Shooter.isAtTarget()", shooter.isAtTarget());
        Logger.recordOutput("AutoFeed/Turret.isAtTarget", turret.isAtTarget());
        if (shooter.isAtTarget() && turret.isAtTarget()) {
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
        indexer.setSpindexerSpeed(IndexerStates.OFF.spindexer);
        indexer.setTowerSpeed(IndexerStates.OFF.tower);
    }

}