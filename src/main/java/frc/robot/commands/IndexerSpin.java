package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IndexerSubsystem.IndexerStates;

public class IndexerSpin extends Command {
    private IndexerSubsystem Indexer;
    private IndexerStates State;

    public IndexerSpin(IndexerSubsystem IndexerSubsystem, IndexerStates state) {
        this.Indexer = IndexerSubsystem;
        this.State = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IndexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Indexer.setIndexerSpeed(State.getSpindexer());
        Indexer.setTowerSpeed(State.getTower());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Indexer.setIndexerSpeed(IndexerStates.OFF.getSpindexer());
        Indexer.setTowerSpeed(IndexerStates.OFF.getTower());
    }

}
