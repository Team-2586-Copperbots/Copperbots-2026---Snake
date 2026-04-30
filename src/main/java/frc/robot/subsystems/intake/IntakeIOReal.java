package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.Constants.CANIds;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXLoggableInputs;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX wristMotor, rollerMotor;
    private final TalonFXLoggableInputs wristInputs, rollerInputs;
    private final CANcoder cancoder;

    private final TalonFXConfiguration wristMotorConfig, rollerMotorConfig;

    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private IntakePosition targetPosition = IntakePosition.IN;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeIOReal() {
        wristMotor = new TalonFX(CANIds.INTAKE_WRIST_MOTOR, Canivore);
        rollerMotor = new TalonFX(CANIds.INTAKE_ROLLER_MOTOR, Canivore);
        cancoder = new CANcoder(CANIds.INTAKE_CANCODER, Canivore);

        wristMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig = new TalonFXConfiguration();

        wristMotorConfig.CurrentLimits.StatorCurrentLimit = 45;
        wristMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        wristMotorConfig.Feedback.FeedbackRemoteSensorID = CANIds.INTAKE_CANCODER;
        wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // when in tolerence no pid
        wristMotorConfig.ClosedLoopGeneral.GainSchedErrorThreshold = 0.015;
        wristMotorConfig.Slot0.GainSchedBehavior = GainSchedBehaviorValue.Inactive;

        var pidConfig = wristMotorConfig.Slot0;
        pidConfig.kP = 18.000;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;

        // friction
        pidConfig.kS = 0.300;
        pidConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        // gravity
        pidConfig.kG = 0.3750;
        pidConfig.GravityType = GravityTypeValue.Arm_Cosine;
        pidConfig.GravityArmPositionOffset = 0.131;

        rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;

        wristMotor.getConfigurator().apply(wristMotorConfig);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);

        wristInputs = new TalonFXLoggableInputs(wristMotor);
        rollerInputs = new TalonFXLoggableInputs(rollerMotor);
    }

    @Override
    public void updateInputs() {

        inputs.currentCancoderPosition = cancoder.getPosition().getValueAsDouble();
        inputs.tagertPosition = targetPosition;

        Logger.processInputs("Intake", inputs);
        wristInputs.log("Intake/Wrist Motor");
        rollerInputs.log("Intake/Roller Motor");
    }

    @Override
    public IntakeIOInputsAutoLogged getInputs() {
        return inputs;
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.INTAKE_WRIST_MOTOR:
                return wristInputs.getInputs();
            case CANIds.INTAKE_ROLLER_MOTOR:
                return rollerInputs.getInputs();
            default:
                return null;
        }
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void setWristPositionTarget(IntakePosition position) {
        targetPosition = position;
        wristMotor.setControl(positionVoltage.withPosition(position.value));
    }

    @Override
    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }
}
