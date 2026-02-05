package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX indexerMotor;
    private final TalonFX towerMotor;
    private final TalonFXConfiguration indexerMotorConfig;
    private final TalonFXConfiguration towerMotorConfig;

    public IndexerSubsystem() {
        indexerMotor = new TalonFX(CANIds.INDEXER_MOTOR);
        towerMotor = new TalonFX(CANIds.TOWER_MOTOR);

        indexerMotorConfig = new TalonFXConfiguration();
        towerMotorConfig = new TalonFXConfiguration();

        indexerMotor.getConfigurator().apply(indexerMotorConfig);
        towerMotor.getConfigurator().apply(towerMotorConfig);
    }

    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }

    public void setTowerSpeed(double speed) {
        towerMotor.set(speed);
    }


}
