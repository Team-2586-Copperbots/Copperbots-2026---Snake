package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import frc.robot.Constants.INTAKE_CONSTANTS;
import frc.robot.subsystems.intake.Intake.IntakePosition;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.CANIds.Canivore;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX wristMotor;
    private final TalonFX rollerMotor;
    private final CANcoder cancoder;

    private final TalonFXConfiguration wristMotorConfig;
    private final TalonFXConfiguration rollerMotorConfig;
    private final CANcoderConfiguration coancoderConfig;

    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private IntakePosition targetPosition = IntakePosition.IN;
    private boolean isClosedLoop;

    public IntakeIOReal() {
        wristMotor = new TalonFX(Constants.CANIds.INTAKE_WRIST_MOTOR, Canivore);
        rollerMotor = new TalonFX(Constants.CANIds.INTAKE_ROLLER_MOTOR, Canivore);
        cancoder = new CANcoder(Constants.CANIds.INTAKE_CANCODER, Canivore);

        wristMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig = new TalonFXConfiguration();
        coancoderConfig = new CANcoderConfiguration();

        coancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        wristMotorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        wristMotorConfig.Feedback.RotorToSensorRatio = Constants.INTAKE_CONSTANTS.rotorToIntake;
        wristMotorConfig.Feedback.SensorToMechanismRatio = 1;

        var motorOutputConfigs = wristMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var pidConfig = wristMotorConfig.Slot0;
        // TODO: tune pid at all!?
        pidConfig.kP = 0.6;
        pidConfig.kI = 0.05;
        pidConfig.kD = 0.00;

        wristMotor.getConfigurator().apply(wristMotorConfig);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
        cancoder.getConfigurator().apply(coancoderConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.currentRollerSpeed = rollerMotor.getVelocity().getValueAsDouble();
        setWristPositionFromCancoder();
        inputs.currentWristPosition = wristMotor.getPosition().getValueAsDouble();
        inputs.currentCancoderPosition = cancoder.getPosition().getValueAsDouble();

        inputs.wristSetpoint = targetPosition;
        inputs.percentageWristSpeed = wristMotor.get();
        inputs.rollerSetpoint = rollerMotor.get();
        inputs.isClosedLoop = isClosedLoop;
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

    @Override
    public void setWristPositionFromCancoder() {
        wristMotor.setPosition(cancoder.getPosition().getValue().in(Rotations) * INTAKE_CONSTANTS.rotorToIntake);
    }

    @Override
    public void setWristSpeed(double speed) {
        isClosedLoop = false;
        wristMotor.set(speed);
    }
}
