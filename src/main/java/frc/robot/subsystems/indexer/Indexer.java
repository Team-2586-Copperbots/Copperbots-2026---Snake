package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.*;

public class Indexer extends SubsystemBase {
    private IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

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
        UP(-0.3, 0.4),
        DOWN(0.3, -0.4),
        OFF(0, 0);

        private final double spindexer;
        private final double tower;

        private IndexerStates(double spindexer, double tower) {
            this.spindexer = spindexer;
            this.tower = tower;
        }

        public double getSpindexer() {
            return spindexer;
        }

        public double getTower() {
            return tower;
        }
    }

}
