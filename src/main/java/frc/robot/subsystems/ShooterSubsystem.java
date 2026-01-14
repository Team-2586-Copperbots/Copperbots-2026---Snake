package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ShooterSubsystem extends SubsystemBase {
    TalonFX shooterMotor;
    TalonFX shooterMotor2;
    TalonFXConfiguration shooterConfig;
    TalonFXConfiguration shooterConfig2;
    private double shooterSpeed;

    public ShooterSubsystem() {

        shooterMotor = new TalonFX(CANIds.ShooterMotor1CANID);
        shooterMotor2 = new TalonFX(CANIds.ShooterMotor2CANID);

        shooterConfig = new TalonFXConfiguration();
        shooterConfig2 = new TalonFXConfiguration();

        shooterMotor.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig2);

        shooterMotor2.setControl(new Follower(CANIds.ShooterMotor1CANID, null));
    }

    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
    }

    public void setTargetShooterSpeed(double speed) {
        shooterSpeed = speed;
    }

    public void increaseShooterSpeed() {
        if (shooterSpeed < 1.0) {
            shooterSpeed += 0.05;
        }
    }

    public void decreaseShooterSpeed() {
        if (shooterSpeed > 0.0) {
            shooterSpeed -= 0.05;
        }
    }

    public double getShooterSpeed() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public double getTargetShooterSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getTargetShooterSpeed'");
    }
}
