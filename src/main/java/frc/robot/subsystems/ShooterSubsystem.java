package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.SHOOTER_CONSTANTS.*;

public class ShooterSubsystem extends SubsystemBase {
    // motors
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;

    // config vars
    private final TalonFXConfiguration shooterConfig;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(0);

    public ShooterSubsystem() {

        shooterMotor1 = new TalonFX(SHOOTER_MOTOR_1_ID);
        shooterMotor2 = new TalonFX(SHOOTER_MOTOR_2_ID);

        shooterConfig = new TalonFXConfiguration();

        var motorOutputConfigs = shooterConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var pidConfig = shooterConfig.Slot0;
        pidConfig.kP = 0.000;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;
        pidConfig.kV = 0.110;
        pidConfig.kS = 0.050;

        shooterMotor1.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig);
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    /** Set and forget command to change the shooter's speed */
    public Command setShooterSpeedCommand(double wheelSpeed) {
        return runOnce(() -> {
            shooterMotor1.setControl(velocityVoltage.withVelocity(wheelSpeed));
        });
    }

    public void setShooterSpeed(double speed) {
        shooterMotor1.setControl(velocityVoltage.withVelocity(speed));
    }

    /** run end command to run the shooter at a speed and set to zero upon ending */
    public Command runEndShooterSpeed(double speed) {
        return runEnd(() -> {
            shooterMotor1.setControl(velocityVoltage.withVelocity(speed));
        }, () -> {
            shooterMotor1.setControl(velocityVoltage.withVelocity(0.0));
        });
    }

    /**
     * run end command to run the shooter at a speed and set to default speed upon
     * ending
     */
    public Command increaseShooterSpeedTemp(double speed) {
        return runEnd(() -> {
            shooterMotor1.setControl(velocityVoltage.withVelocity(speed));
        }, () -> {
            shooterMotor1.setControl(velocityVoltage.withVelocity(SHOOTER_SPEED));
        });
    }

    public double getMotor1Speed() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    public double getMotor2Speed() {
        return shooterMotor2.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter setpoint", velocityVoltage.Velocity);
        SmartDashboard.putNumber("ShooterSpeed", getMotor1Speed());
        SmartDashboard.putNumber("ahooter motor 2", getMotor2Speed());
        SmartDashboard.putNumber("shotter current", shooterMotor1.getStatorCurrent().getValueAsDouble());
    }
}
