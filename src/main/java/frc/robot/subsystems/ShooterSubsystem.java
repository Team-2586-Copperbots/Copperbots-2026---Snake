package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    // motors
    private final TalonFX shooterMotor;
    private final TalonFX shooterMotor2;

    // config vars
    private final TalonFXConfiguration shooterConfig;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(0);

    public ShooterSubsystem() {

        shooterMotor = new TalonFX(SHOOTER_MOTOR_1_ID);
        shooterMotor2 = new TalonFX(SHOOTER_MOTOR_2_ID);

        shooterConfig = new TalonFXConfiguration();

        var motorOutputConfigs = shooterConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = motorOutputConfigs.NeutralMode.Coast;

        var pidConfig = shooterConfig.Slot0;
        pidConfig.kP = 0.28;
        pidConfig.kI = 0.0;
        pidConfig.kD = 0.0075;
        pidConfig.kV = 0.11;

        shooterMotor.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig);
        shooterMotor2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /** Set and forget command to change the shooter's speed */
    public Command setShooterSpeed(double speed) {
        return runOnce(() -> {
            shooterMotor.setControl(velocityVoltage.withVelocity(speed));
        });
    }

    /** run end command to run the shooter at a speed and set to zero upon ending */
    public Command runShooterTemp(double speed) {
        return runEnd(() -> {
            shooterMotor.setControl(velocityVoltage.withVelocity(speed));
        }, () -> {
            shooterMotor.setControl(velocityVoltage.withVelocity(0.0));
        });
    }

    /**
     * run end command to run the shooter at a speed and set to default speed upon
     * ending
     */
    public Command increaseShooterSpeedTemp(double speed) {
        return runEnd(() -> {
            shooterMotor.setControl(velocityVoltage.withVelocity(speed));
        }, () -> {
            shooterMotor.setControl(velocityVoltage.withVelocity(SHOOTER_SPEED));
        });
    }

    public double getShooterMotorSpeed() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterSpeed", getShooterMotorSpeed());
    }
}
