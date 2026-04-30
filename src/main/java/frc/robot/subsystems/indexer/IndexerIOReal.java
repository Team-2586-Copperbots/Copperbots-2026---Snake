package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXLoggableInputs;

public class IndexerIOReal implements IndexerIO {

    // hardware objects
    private final TalonFX spindexerMotor, towerMotor;
    private final TalonFXLoggableInputs spindexerInputs, towerInputs;
    private final TalonFXConfiguration motorConfig;

    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerIOReal() {
        spindexerMotor = new TalonFX(Constants.CANIds.INDEXER_SPINDEXER_MOTOR, Constants.CANIds.Canivore);
        towerMotor = new TalonFX(Constants.CANIds.INDEXER_TOWER_MOTOR, Constants.CANIds.Canivore);

        motorConfig = new TalonFXConfiguration();
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        spindexerMotor.getConfigurator().apply(motorConfig);
        towerMotor.getConfigurator().apply(motorConfig);

        spindexerInputs = new TalonFXLoggableInputs(spindexerMotor);
        towerInputs = new TalonFXLoggableInputs(towerMotor);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs() {
        inputs.spindexerSpeed = spindexerMotor.getVelocity().getValueAsDouble();
        inputs.towerSpeed = towerMotor.getVelocity().getValueAsDouble();

        Logger.processInputs("Indexer", inputs);
        towerInputs.log("Indexer/Tower Motor");
        spindexerInputs.log("Indexer/Spindexer Motor");
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.INDEXER_SPINDEXER_MOTOR:
                return spindexerInputs.getInputs();
            case CANIds.INDEXER_TOWER_MOTOR:
                return towerInputs.getInputs();
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
        spindexerMotor.set(output);
    }
}
