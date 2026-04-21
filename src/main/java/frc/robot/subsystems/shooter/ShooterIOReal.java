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

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {
    // motors
    private final TalonFXAutoLogged shooterMotor1, shooterMotor2;

    // config vars
    private final TalonFXConfiguration shooterConfig;
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(0);

    public ShooterIOReal() {
        shooterMotor1 = new TalonFXAutoLogged(Constants.CANIds.SHOOTER_MOTOR_1, Canivore);
        shooterMotor2 = new TalonFXAutoLogged(Constants.CANIds.SHOOTER_MOTOR_2, Canivore);

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
    public void updateInputs() {
        Logger.processInputs("Shooter/Motor 1", shooterMotor1.getInputs());
        Logger.processInputs("Shooter/Motor 2", shooterMotor2.getInputs());
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.SHOOTER_MOTOR_1:
                return shooterMotor1.getInputs();
            case CANIds.SHOOTER_MOTOR_2:
                return shooterMotor2.getInputs();
        }
        return null;
    }

    @Override
    public void runVoltage(double voltage) {
        shooterMotor1.setVoltage(voltage);
    }

    @Override
    public void setMotorSetpoint(double velocity) {
        shooterMotor1.setControl(velocityVoltage.withVelocity(velocity));
    }

    @Override
    public void setPercentageSpeed(double speed) {
        shooterMotor1.set(speed);
    }
}
