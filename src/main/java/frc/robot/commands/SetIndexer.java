package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class SetIndexer extends Command {
    private Indexer Indexer;
    private Double Speed;

    public SetIndexer(Indexer Indexer, double speed) {
        this.Indexer = Indexer;
        this.Speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Indexer.setSpindexerSpeed(Speed);
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
        Indexer.setSpindexerSpeed(0);
        Indexer.setTowerSpeed(0);
    }

}
