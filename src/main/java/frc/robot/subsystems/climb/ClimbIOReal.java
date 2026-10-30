package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class ClimbIOReal implements ClimbIO {
    // Note: Climb motors are physically wired to the RoboRIO internal CAN loop
    private final TalonFXAutoLogged climbMotor1, climbMotor2;
    private final BaseStatusSignal[] allClimbSignals;
    private final DigitalInput limitSwitch;
    private final TalonFXConfiguration climbMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private ClimbPosition targePosition = ClimbPosition.DOWN;

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public ClimbIOReal() {
        climbMotor1 = new TalonFXAutoLogged(CANIds.CLIMB_MOTOR_1);
        climbMotor2 = new TalonFXAutoLogged(CANIds.CLIMB_MOTOR_2);

        BaseStatusSignal[] c1 = climbMotor1.getSignals();
        BaseStatusSignal[] c2 = climbMotor2.getSignals();
        allClimbSignals = new BaseStatusSignal[c1.length + c2.length];
        System.arraycopy(c1, 0, allClimbSignals, 0, c1.length);
        System.arraycopy(c2, 0, allClimbSignals, c1.length, c2.length);

        limitSwitch = new DigitalInput(DIO_IDS.CLIMB_LIMIT_SWITCH);
        climbMotorConfig = new TalonFXConfiguration();

        climbMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        climbMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        var motorOutputConfigs = climbMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var pidConfig = climbMotorConfig.Slot0;
        pidConfig.kP = 2.000;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;

        climbMotor1.getConfigurator().apply(climbMotorConfig);
        climbMotor2.getConfigurator().apply(climbMotorConfig);

        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateAndLogInputs() {
        // Refresh all climb signals on the RIO bus in a single batched native call
        BaseStatusSignal.refreshAll(allClimbSignals);
        climbMotor1.updateInputsFromRefreshedSignals();
        climbMotor2.updateInputsFromRefreshedSignals();

        inputs.limitSwitch = !limitSwitch.get();
        inputs.targetPosition = targePosition;

        Logger.processInputs("Climb", inputs);

        Logger.processInputs("Climb/Climb Motor 1", climbMotor1.getInputs());
        Logger.processInputs("Climb/Climb motor 2", climbMotor2.getInputs());
    }

    @Override
    public ClimbIOInputsAutoLogged getInputs() {
        return inputs;
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.CLIMB_MOTOR_1:
                return climbMotor1.getInputs();
            case CANIds.CLIMB_MOTOR_2:
                return climbMotor2.getInputs();
            default:
                return null;
        }
    }

    @Override
    public void setPosition(double position) {
        climbMotor1.setPosition(position);
        climbMotor2.setPosition(position);
    }

    @Override
    public void setSpeed(double speed) {
        climbMotor1.set(speed);
    }

    @Override
    public void setTargetPosition(ClimbPosition targetPosition) {
        targePosition = targetPosition;
        climbMotor1.setControl(positionVoltage.withPosition(targetPosition.value));
    }
}
