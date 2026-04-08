package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Indexer extends SubsystemBase {
    private static Indexer instance = null;
    private IndexerIO io;

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }

    private Indexer() {
        switch (Constants.currentMode) {
                case REAL:
                    io = new IndexerIOReal();
                    break;
                case SIM:
                    io = new IndexerIOSim();
                    break;
                default:
                    io = new IndexerIO() {
                    };
                    break;
            }
    }

    @Override
    public void periodic() {
        io.updateInputs();
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
