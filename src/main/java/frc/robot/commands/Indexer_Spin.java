package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerStates;

public class Indexer_Spin extends Command {
    private Indexer Indexer;
    private IndexerStates State;
    // private boolean simPojectil = false;
    // private int counter = 0;
    // @AutoLogOutput (key="Indexer/timebetwen")
    // private int timeBetwen = 25;

    public Indexer_Spin(Indexer IndexerSubsystem, IndexerStates state) {
        this.Indexer = IndexerSubsystem;
        this.State = state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IndexerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // if (State != IndexerStates.OFF && Constants.currentMode == Constants.Mode.SIM) {
        //     simPojectil = true;
        // }
        Indexer.setSpindexerSpeed(State.spindexer);
        Indexer.setTowerSpeed(State.tower);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if (simPojectil) {
        //     if (counter < timeBetwen) {
        //         counter++;
        //     } else {
        //         counter = 0;
        //         simsProjectile.shootLemmon();
        //     }
        // }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Indexer.setSpindexerSpeed(IndexerStates.OFF.spindexer);
        Indexer.setTowerSpeed(IndexerStates.OFF.tower);
    }

}
