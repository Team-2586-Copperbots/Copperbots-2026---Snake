package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class IndexerIOReal implements IndexerIO {

    // hardware objects
    private final TalonFXAutoLogged indexerMotor, towerMotor;
    private final TalonFXConfiguration motorConfig;

    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final 

    public IndexerIOReal() {
        indexerMotor = new TalonFXAutoLogged(Constants.CANIds.SPINDEXER_INDEXER_MOTOR, Constants.CANIds.Canivore);
        towerMotor = new TalonFXAutoLogged(Constants.CANIds.SPINDEXER_TOWER_MOTOR, Constants.CANIds.Canivore);

        motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        indexerMotor.getConfigurator().apply(motorConfig);
        towerMotor.getConfigurator().apply(motorConfig);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs() {
        inputs.spindexerSpeed = indexerMotor.getVelocity().getValueAsDouble();
        inputs.towerSpeed = towerMotor.getVelocity().getValueAsDouble();

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
