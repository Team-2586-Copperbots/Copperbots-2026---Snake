package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import frc.robot.Constants.SupplyLimmits;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXLoggableInputs;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {
    // motors
    private final TalonFX motor1, motor2;
    private final TalonFXLoggableInputs motor1Inputs, motor2Inputs;

    // config vars
    private final TalonFXConfiguration shooterConfig;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(0);

    public ShooterIOReal() {
        motor1 = new TalonFX(Constants.CANIds.SHOOTER_MOTOR_1, Canivore);
        motor2 = new TalonFX(Constants.CANIds.SHOOTER_MOTOR_2, Canivore);
        shooterConfig = new TalonFXConfiguration();

        shooterConfig.CurrentLimits.StatorCurrentLimit = SHOOTER_CONSTANTS.STATOR_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = SupplyLimmits.SHOOTER;

        var motorOutputConfigs = shooterConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var pidConfig = shooterConfig.Slot0;
        pidConfig.kP = 0.500;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;
        pidConfig.kV = 0.110;
        pidConfig.kS = 0.050;

        motor1.getConfigurator().apply(shooterConfig);
        motor2.getConfigurator().apply(shooterConfig);
        motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed));

        motor1Inputs = new TalonFXLoggableInputs(motor1);
        motor2Inputs = new TalonFXLoggableInputs(motor2);
    }

    @Override
    public void updateInputs() {
        motor1Inputs.log("Shooter/Motor 1");
        motor2Inputs.log("Shooter/Motor 2");
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.SHOOTER_MOTOR_1:
                return motor1Inputs.getInputs();
            case CANIds.SHOOTER_MOTOR_2:
                return motor2Inputs.getInputs();
        }
        return null;
    }

    @Override
    public void runVoltage(double voltage) {
        motor1.setVoltage(voltage);
    }

    @Override
    public void setMotorSetpoint(double velocity) {
        motor1.setControl(velocityVoltage.withVelocity(velocity));
    }

    @Override
    public void setPercentageSpeed(double speed) {
        motor1.set(speed);
    }
}
