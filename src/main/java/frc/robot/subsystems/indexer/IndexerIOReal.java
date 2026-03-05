package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class IndexerIOReal implements IndexerIO{

    // hardware objects
    private final TalonFX indexerMotor;
    private final TalonFX towerMotor;
    private final TalonFXConfiguration motorConfig;

    public IndexerIOReal() {
        indexerMotor = new TalonFX(Constants.CANIds.INDEXER_MOTOR, Constants.CANIds.Canivore);
        towerMotor = new TalonFX(Constants.CANIds.TOWER_MOTOR, Constants.CANIds.Canivore);

        motorConfig = new TalonFXConfiguration();
        indexerMotor.getConfigurator().apply(motorConfig);
        towerMotor.getConfigurator().apply(motorConfig);

    }

    /** Updates the set of loggable inputs. */
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.spindexerSpeed = indexerMotor.getVelocity().getValueAsDouble();
        inputs.towerSpeed = towerMotor.getVelocity().getValueAsDouble();
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
