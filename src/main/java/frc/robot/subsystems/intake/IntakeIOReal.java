package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.subsystems.intake.Intake.IntakePosition;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX wristMotor;
    private final TalonFX rollerMotor;
    private final CANcoder cancoder;

    private final TalonFXConfiguration wristMotorConfig;
    private final TalonFXConfiguration rollerMotorConfig;
    private double applyedVoltsMkS = 0;

    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private IntakePosition targetPosition = IntakePosition.IN;
    private boolean isClosedLoop;

    public IntakeIOReal() {
        wristMotor = new TalonFX(CANIds.INTAKE_WRIST_MOTOR, Canivore);
        rollerMotor = new TalonFX(CANIds.INTAKE_ROLLER_MOTOR, Canivore);
        cancoder = new CANcoder(CANIds.INTAKE_CANCODER, Canivore);

        wristMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig = new TalonFXConfiguration();

        // wristMotorConfig.CurrentLimits.StatorCurrentLimit = 70;

        wristMotorConfig.Feedback.FeedbackRemoteSensorID = CANIds.INTAKE_CANCODER;
        wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        wristMotorConfig.ClosedLoopGeneral.GainSchedErrorThreshold = 0.01;

        var pidConfig = wristMotorConfig.Slot0;
        // TODO: tune pid at all?
        pidConfig.kP = 13.000;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;

        // friction
        pidConfig.kS = 0.320;
        pidConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        // gravity
        pidConfig.kG = 0.3750;
        pidConfig.GravityType = GravityTypeValue.Arm_Cosine;
        pidConfig.GravityArmPositionOffset = 0.131;
        // when in tolerence no pid
        pidConfig.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;

        wristMotor.getConfigurator().apply(wristMotorConfig);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.wristIsOK = wristMotor.isAlive();
        inputs.currentWristPosition = wristMotor.getPosition().getValueAsDouble();
        inputs.wristSetpoint = targetPosition;
        if (wristMotor.get() > 0) {
            applyedVoltsMkS = wristMotor.getMotorVoltage().getValueAsDouble() - wristMotorConfig.Slot0.kS;
        } else {
            applyedVoltsMkS = wristMotor.getMotorVoltage().getValueAsDouble() + wristMotorConfig.Slot0.kS;
        }
        inputs.wristVolts = applyedVoltsMkS;
        inputs.wristAmps = wristMotor.getStatorCurrent().getValueAsDouble();
        inputs.currentRollerSpeed = rollerMotor.getVelocity().getValueAsDouble();

        inputs.currentCancoderPosition = cancoder.getPosition().getValueAsDouble();

        inputs.rollerSetpoint = rollerMotor.get();
        inputs.wristIsClosedLoop = isClosedLoop;
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void setWristPositionTarget(IntakePosition position) {
        isClosedLoop = true;
        targetPosition = position;
        wristMotor.setControl(positionVoltage.withPosition(position.value));
    }

    // @Override
    // public void setWristPositionFromCancoder() {
    // wristMotor.setPosition(cancoder.getPosition().getValue().in(Rotations) *
    // INTAKE_CONSTANTS.rotorToIntake);
    // }

    @Override
    public void setWristSpeed(double speed) {
        isClosedLoop = false;
        wristMotor.set(speed);
    }
}
