package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.util.auto_logging_stuff.LoggedTalonFXInputs;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class ClimbIOReal implements ClimbIO {
    private final TalonFXAutoLogged climbMotor1, climbMotor2;
    // private final LoggedTalonFX climb1, climb2;
    private final DigitalInput limitSwitch;
    private final TalonFXConfiguration climbMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private ClimbPosition targePosition = ClimbPosition.DOWN;
    // private final SparkMax motot = new SparkMax(0, MotorType.kBrushless)
    // private final PositionTorqueCurrentFOC

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    // private final MotorIOTalon climbMotor1IO, climbMotor2IO;
    // private final MotorIOInputsAutoLogged climbMotor1Inputs = new
    // MotorIOInputsAutoLogged();
    // private final MotorIOInputsAutoLogged climbMotor2Inputs = new
    // MotorIOInputsAutoLogged();

    public ClimbIOReal() {
        climbMotor1 = new TalonFXAutoLogged(CANIds.CLIMB_MOTOR_1);
        climbMotor2 = new TalonFXAutoLogged(CANIds.CLIMB_MOTOR_2);
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

        climbMotor1.getConfigurator().apply(climbMotorConfig);
        climbMotor2.getConfigurator().apply(climbMotorConfig);

        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), MotorAlignmentValue.Aligned));

        // climbMotor1IO = new MotorIOTalon(climbMotor1);
        // climbMotor2IO = new MotorIOTalon(climbMotor2);
    }

    @Override
    public void updateAndLogInputs() {
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
