package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.GeneralUtils;

public class Indexer_AutoSpeed extends Command {
    private Indexer indexer;
    private Shooter shooter = Shooter.getInstance();
    private Turret turret = Turret.getInstance();

    public Indexer_AutoSpeed(Indexer indexerSubsystem) {
        this.indexer = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {

    }

    public boolean shouldShoot() {
        // return (shooter.isAtTarget() && turret.isAtTarget());
        return true;
    }

    @Override
    public void execute() {
        Logger.recordOutput("AutoFeed/Shooter.isAtTarget()", shooter.isAtTarget());
        Logger.recordOutput("AutoFeed/Turret.isAtTarget", turret.isAtTarget());
        if (shouldShoot()) {
            Logger.recordOutput("AutoFeed/autoindexer", GeneralUtils.getAutoIndexerState());
            indexer.setSpindexerSpeed(GeneralUtils.getAutoIndexerState().spindexer());
            indexer.setTowerSpeed(GeneralUtils.getAutoIndexerState().tower());
        } else {
            indexer.setSpindexerSpeed(IndexerStates.OFF.spindexer());
            indexer.setTowerSpeed(IndexerStates.OFF.tower());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        indexer.setSpindexerSpeed(IndexerStates.OFF.spindexer());
        indexer.setTowerSpeed(IndexerStates.OFF.tower());
    }
}
