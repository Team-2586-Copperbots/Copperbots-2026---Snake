package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.auto_logging_stuff.SimMotorAutoLogged;

public class IndexerIOSim implements IndexerIO {
    private SimMotorAutoLogged towerMotor;
    private SimMotorAutoLogged spindexerMotor;

    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerIOSim() {
        towerMotor = new SimMotorAutoLogged();
        spindexerMotor = new SimMotorAutoLogged();

    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs() {
        inputs.spindexerSpeed = spindexerMotor.getInputs().velocity / 100;
        inputs.towerSpeed = towerMotor.getInputs().velocity / 100;
        Logger.processInputs("Spindexer", inputs);
        Logger.processInputs("Spindexer/tower motor", towerMotor.getInputs());
        Logger.processInputs("Spindexer/spindexer motor", spindexerMotor.getInputs());
    }

    @Override
    public void setTowerSpeed(double output) {
        towerMotor.setSimSpeed(output);
    }

    @Override
    public void setSpindexerSpeed(double output) {
        spindexerMotor.setSimSpeed(output);
    }
}
