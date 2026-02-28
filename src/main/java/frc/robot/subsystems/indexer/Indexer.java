package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import org.littletonrobotics.junction.*;
import static frc.robot.Constants.CANIds.Canivore;

public class Indexer extends SubsystemBase {
    private IndexerIO io;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("indexer inputs", inputs);
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
        UP(0.2, 0.4),
        TOWER(0, 0.4),
        DOWN(-0.3, -0.4),
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
