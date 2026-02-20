package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import static frc.robot.Constants.CANIds.Canivore;

public class ShooterSubsystem extends SubsystemBase {
    // motors
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private double speedForPeriodicShooter = 10;

    // config vars
    private final TalonFXConfiguration shooterConfig;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(0);

    public ShooterSubsystem() {

        shooterMotor1 = new TalonFX(CANIds.SHOOTER_MOTOR_1_ID, Canivore);
        shooterMotor2 = new TalonFX(CANIds.SHOOTER_MOTOR_2_ID, Canivore);

        shooterConfig = new TalonFXConfiguration();

        var motorOutputConfigs = shooterConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var pidConfig = shooterConfig.Slot0;
        pidConfig.kP = 0.500;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;
        pidConfig.kV = 0.110;
        pidConfig.kS = 0.050;

        shooterMotor1.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig);
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    

    // negative to decrese
    public void setShooterSpeedAjust(double amount) {
        speedForPeriodicShooter += amount;
    }

    // sets the absolute speed
    public void setShooterSpeedSet(double setPoint) {
        speedForPeriodicShooter = setPoint;
    }    

    

    

    public double getMotor1Speed() {
        return shooterMotor1.getVelocity().getValueAsDouble();
    }

    public double getMotor2Speed() {
        return shooterMotor2.getVelocity().getValueAsDouble();
    }

    
    @Override
    public void periodic() {
        // TODO: posibly use diffrent periodic:
        // if (oldValue != newValue) {
        //   oldValue = newValue;
        //   setcontrol(newValue)
        // }
        // intent: reduce CAN lode if not needed

        // TODO: look at CAN load
        shooterMotor1.setControl(velocityVoltage.withVelocity(speedForPeriodicShooter));
        
        SmartDashboard.putNumber("Shooter velocityVoltage.Velocity", velocityVoltage.Velocity);
        SmartDashboard.putNumber("dynamic speed", speedForPeriodicShooter);
        SmartDashboard.putNumber("ShooterSpeed", getMotor1Speed());
        SmartDashboard.putNumber("shotter current", shooterMotor1.getStatorCurrent().getValueAsDouble());
    }
}
