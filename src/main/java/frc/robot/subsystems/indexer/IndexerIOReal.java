package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.util.auto_loggint_stuff.MotorIO;
import frc.robot.util.auto_loggint_stuff.MotorIOInputsAutoLogged;
import frc.robot.util.auto_loggint_stuff.MotorIOTalon;

public class IndexerIOReal implements IndexerIO {

    // hardware objects
    private final TalonFX indexerMotor;
    private final TalonFX towerMotor;
    private final TalonFXConfiguration motorConfig;

    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final MotorIO towerMotorIO, indexerMotoIO;
    private final MotorIOInputsAutoLogged towerMotorInputs = new MotorIOInputsAutoLogged();
    private final MotorIOInputsAutoLogged indexerMotorInputs = new MotorIOInputsAutoLogged();

    public IndexerIOReal() {
        indexerMotor = new TalonFX(Constants.CANIds.SPINDEXER_INDEXER_MOTOR, Constants.CANIds.Canivore);
        towerMotor = new TalonFX(Constants.CANIds.SPINDEXER_TOWER_MOTOR, Constants.CANIds.Canivore);

        motorConfig = new TalonFXConfiguration();
        indexerMotor.getConfigurator().apply(motorConfig);
        towerMotor.getConfigurator().apply(motorConfig);

        indexerMotoIO = new MotorIOTalon(indexerMotor);
        towerMotorIO = new MotorIOTalon(towerMotor);
    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs() {
        inputs.spindexerSpeed = indexerMotor.getVelocity().getValueAsDouble();
        inputs.towerSpeed = towerMotor.getVelocity().getValueAsDouble();

        Logger.processInputs("Indexer", inputs);
        indexerMotoIO.updateInputs(indexerMotorInputs);
        towerMotorIO.updateInputs(towerMotorInputs);
        Logger.processInputs("Indexer/IndexerMotor", indexerMotorInputs);
        Logger.processInputs("Indexer/TowerMotor", towerMotorInputs);
    }

    @Override
    public MotorIOInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.SPINDEXER_INDEXER_MOTOR:
                return indexerMotorInputs;
            case CANIds.SPINDEXER_TOWER_MOTOR:
                return towerMotorInputs;
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
