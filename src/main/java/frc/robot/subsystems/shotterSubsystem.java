package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class shotterSubsystem {
    TalonFX shooterMotor;
    TalonFX shooterMotor2;
    TalonFXConfiguration shooterConfig;

    public shotterSubsystem() {

        shooterMotor = new TalonFX(10);
        shooterMotor2 = new TalonFX(11);
        shooterConfig = new TalonFXConfiguration();
        
        //FIXME: fix CAN ideas

        
    }

    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }
}
