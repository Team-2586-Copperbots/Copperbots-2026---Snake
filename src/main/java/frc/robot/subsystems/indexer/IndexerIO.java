package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        // both are in the range of 1.0 - 0.0
        public double towerSpeed = 0;
        public double spindexerSpeed = 0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setTowerSpeed(double output) {}

    public default void setSpindexerSpeed(double output) {}
}
