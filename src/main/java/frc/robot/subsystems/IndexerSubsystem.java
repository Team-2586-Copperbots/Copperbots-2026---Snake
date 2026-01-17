package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX indexerMotor;
    private final TalonFXConfiguration indexerMotorConfig;
    public IndexerSubsystem() {
        indexerMotor = new TalonFX(0);
        indexerMotorConfig = new TalonFXConfiguration();
        indexerMotor.getConfigurator().apply(indexerMotorConfig);
    }

}
