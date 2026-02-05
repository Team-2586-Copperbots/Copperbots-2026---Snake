package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class SetIndexer extends Command {
    private IndexerSubsystem Indexer;
    private Double Speed;

    public SetIndexer(IndexerSubsystem Indexer, double speed) {
        this.Indexer = Indexer;
        this.Speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Indexer.setIndexerSpeed(Speed);
        Indexer.setTowerSpeed(Speed);
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
        Indexer.setIndexerSpeed(0);
        Indexer.setTowerSpeed(0);
    }

}
