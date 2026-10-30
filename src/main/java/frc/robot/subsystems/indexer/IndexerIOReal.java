package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class IndexerIOReal implements IndexerIO {

    // hardware objects
    private final TalonFXAutoLogged indexerMotor, towerMotor;
    private final BaseStatusSignal[] allIndexerSignals;
    private final TalonFXConfiguration motorConfig;

    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerIOReal() {
        indexerMotor = new TalonFXAutoLogged(Constants.CANIds.SPINDEXER_INDEXER_MOTOR, Constants.CANIds.Canivore);
        towerMotor = new TalonFXAutoLogged(Constants.CANIds.SPINDEXER_TOWER_MOTOR, Constants.CANIds.Canivore);

        BaseStatusSignal[] s1 = indexerMotor.getSignals();
        BaseStatusSignal[] s2 = towerMotor.getSignals();
        allIndexerSignals = new BaseStatusSignal[s1.length + s2.length];
        System.arraycopy(s1, 0, allIndexerSignals, 0, s1.length);
        System.arraycopy(s2, 0, allIndexerSignals, s1.length, s2.length);

        motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.StatorCurrentLimit = 80;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        indexerMotor.getConfigurator().apply(motorConfig);
        towerMotor.getConfigurator().apply(motorConfig);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs() {
        // Batch refresh both indexer motors in a single native call
        BaseStatusSignal.refreshAll(allIndexerSignals);
        indexerMotor.updateInputsFromRefreshedSignals();
        towerMotor.updateInputsFromRefreshedSignals();

        // TalonFXInputsAutoLogged indexerInputs = indexerMotor.getInputs();
        // TalonFXInputsAutoLogged towerInputs = towerMotor.getInputs();

        inputs.spindexerSpeed = indexerMotor.getInputs().velocity;
        inputs.towerSpeed = towerMotor.getInputs().velocity;

        Logger.processInputs("Indexer", inputs);
        Logger.processInputs("Indexer/IndexerMotor", indexerMotor.getInputs());
        Logger.processInputs("Indexer/TowerMotor", towerMotor.getInputs());
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.SPINDEXER_INDEXER_MOTOR:
                return indexerMotor.getInputs();
            case CANIds.SPINDEXER_TOWER_MOTOR:
                return towerMotor.getInputs();
            default:
                return null;
        }
    }

    @Override
    public void setTowerSpeed(double output) {
        towerMotor.set(output);
    }

    @Override
    public void setSpindexerSpeed(double output) {
        indexerMotor.set(output);
    }
}
