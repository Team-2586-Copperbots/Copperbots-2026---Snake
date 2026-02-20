package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import static frc.robot.Constants.CANIds.Canivore;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX climbMotor1, climbMotor2;
    private final TalonFXConfiguration climbMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    public ClimbSubsystem() {
        climbMotor1 = new TalonFX(CANIds.CLIMB_MOTOR_1, Canivore);
        climbMotor2 = new TalonFX(CANIds.CLIMB_MOTOR_2, Canivore);

        climbMotor1Config = new TalonFXConfiguration();
        climbMotor2Config = new TalonFXConfiguration();

        var motorOutputConfigs = shooterConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        climbMotor1.getConfigurator().apply(climbMotor1Config);
        climbMotor2.getConfigurator().apply(climbMotor2Config);

        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void setClimbSpeed(double speed) {
        climbMotor1.set(speed);
    }

}
