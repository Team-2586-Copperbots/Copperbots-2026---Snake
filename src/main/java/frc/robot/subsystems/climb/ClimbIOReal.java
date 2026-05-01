package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXLoggableInputs;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX climbMotor;
    private final TalonFXConfiguration climbMotorConfig;
    private final TalonFXLoggableInputs climbMotorInputs;
    private final DigitalInput limitSwitch;
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true);
    private ClimbPosition targePosition = ClimbPosition.DOWN;
    // private final PositionTorqueCurrentFOC

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public ClimbIOReal() {
        climbMotor = new TalonFX(CANIds.CLIMB_MOTOR_1);
        limitSwitch = new DigitalInput(DIO_IDS.CLIMB_LIMIT_SWITCH);
        climbMotorConfig = new TalonFXConfiguration();

        // climbMotorConfig.CurrentLimits.StatorCurrentLimit = 80; un necicary

        var motorOutputConfigs = climbMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var pidConfig = climbMotorConfig.Slot0;
        pidConfig.kP = 2.000;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;

        // climbMotorConfig.CurrentLimits.SupplyCurrentLimit = 80;

        climbMotor.getConfigurator().apply(climbMotorConfig);

        climbMotorInputs = new TalonFXLoggableInputs(climbMotor);
    }

    @Override
    public void updateAndLogInputs() {
        inputs.limitSwitch = limitSwitch.get();
        inputs.targetPosition = targePosition;

        Logger.processInputs("Climb", inputs);
        climbMotorInputs.log("Climb/Motor 1");
    }

    @Override
    public ClimbIOInputsAutoLogged getInputs() {
        return inputs;
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.CLIMB_MOTOR_1:
                return climbMotorInputs.getInputs();
            default:
                return null;
        }
    }

    @Override
    public void setPosition(double position) {
        climbMotor.setPosition(position);
    }

    @Override
    public void setSpeed(double speed) {
        climbMotor.set(speed);
    }

    @Override
    public void setTargetPosition(ClimbPosition targetPosition) {
        targePosition = targetPosition;
        climbMotor.setControl(positionVoltage.withPosition(targetPosition.value));
    }
}
