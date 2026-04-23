package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import frc.robot.util.auto_logging_stuff.LoggedTalonFXInputs;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class ShooterIOReal extends ShooterIO {

    // config vars
    private final TalonFXConfiguration shooterConfig;

    public ShooterIOReal() {
        super();

        shooterConfig = new TalonFXConfiguration();

        shooterConfig.CurrentLimits.StatorCurrentLimit = SHOOTER_CONSTANTS.CURRENT_LIMIT;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 50;
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

    @Override
    public void periodic() {
        motor1Inputs.log("Shooter/Motor 1", shooterMotor1);
        motor2Inputs.log("Shooter/Motor 2", shooterMotor2);
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.SHOOTER_MOTOR_1:
                return motor1Inputs.getInputs(shooterMotor1);
            case CANIds.CLIMB_MOTOR_2:
                return motor2Inputs.getInputs(shooterMotor2);
            default:
                return null;
        }
    }
}
