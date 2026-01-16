package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class ShooterSubsystem extends SubsystemBase {
    private TalonFX shooterMotor;
    private TalonFX shooterMotor2;
    private TalonFXConfiguration shooterConfig;
    private TalonFXConfiguration shooterConfig2;
    private double shooterSpeed;
    private ProfiledPIDController PIDController;

    public ShooterSubsystem() {

        shooterMotor = new TalonFX(CANIds.ShooterMotor1CANID);
        shooterMotor2 = new TalonFX(CANIds.ShooterMotor2CANID);

        shooterConfig = new TalonFXConfiguration();
        shooterConfig2 = new TalonFXConfiguration();

        shooterMotor.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig2);

        shooterSpeed = 0.0;

        PIDController = new ProfiledPIDController(0.005, 0.1, 0.0, new Constraints(1000, 2000));
    }

    public void setShooterSpeed(double speed) {
        shooterMotor.set(speed);
        shooterMotor2.set(-speed);
    }

    public void resetPID() {
        PIDController.reset(getShooterMotorSpeed());
    }

    public double getCurrentPosition() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public void setTargetSpeed(double speed) {
        shooterSpeed = speed;
    }

    public double getTargetSpeed() {
        return shooterSpeed;
    }

    public void placeholder(double goal) {

        PIDController.setGoal(goal);
        setShooterSpeed(PIDController.calculate(getCurrentPosition()));

    }

    public double getShooterMotorSpeed() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public double getTargetShooterSpeed() {
        return shooterSpeed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterSpeed", getShooterMotorSpeed());
        SmartDashboard.putNumber("TargetShooterSpeed", shooterSpeed);
    }
}
