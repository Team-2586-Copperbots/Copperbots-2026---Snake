package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexerSpin extends Command {
    private IndexerSubsystem Indexer;

    public IndexerSpin(IndexerSubsystem IndexerSubsystem) {
        this.Indexer = IndexerSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IndexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Indexer.setIndexerSpeed(.30);
        Indexer.setTowerSpeed(.60);
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
