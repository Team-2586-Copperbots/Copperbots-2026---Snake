package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {
    private double towerSpeed = 0;
    private double spindexerSpeed = 0;

    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerIOSim() {

    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs() {
        inputs.spindexerSpeed = spindexerSpeed;
        inputs.towerSpeed = towerSpeed;
    }

    @Override
    public void setTowerSpeed(double output) {
        towerSpeed = output;
    }

    @Override
    public void setSpindexerSpeed(double output) {
        spindexerSpeed = output;
    }
}
