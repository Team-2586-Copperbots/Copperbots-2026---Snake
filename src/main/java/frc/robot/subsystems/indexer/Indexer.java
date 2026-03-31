package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.*;

public class Indexer extends SubsystemBase {
    private static Indexer instance = null;
    private IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public static Indexer getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    instance = new Indexer(new IndexerIOReal());
                    break;
                case SIM:
                    instance = new Indexer(new IndexerIOSim());
                    break;
                default:
                    instance = new Indexer(new IndexerIO() {
                    });
                    break;
            }
        }
        return instance;
    }

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    public Command setSpindexerSpeedCommand(double speed) {
        return runOnce(() -> io.setSpindexerSpeed(speed));
    }

    public void setSpindexerSpeed(double speed) {
        io.setSpindexerSpeed(speed);
    }

    public void setTowerSpeed(double speed) {
        io.setTowerSpeed(speed);
    }

    public static enum IndexerStates {
        ON(-0.45, 0.5),
        OFF(0, 0);

        public final double spindexer;
        public final double tower;

        private IndexerStates(double spindexer, double tower) {
            this.spindexer = spindexer;
            this.tower = tower;
        }

    }

}
