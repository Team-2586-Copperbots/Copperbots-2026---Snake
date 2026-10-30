package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.CANIds;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

public class IntakeIOReal implements IntakeIO {
    private final TalonFXAutoLogged wristMotor, rollerMotor;
    private final CANcoder cancoder;
    private final StatusSignal<Angle> cancoderPosition;
    private final BaseStatusSignal[] allIntakeSignals;

    private final TalonFXConfiguration wristMotorConfig, rollerMotorConfig;

    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private IntakePosition targetPosition = IntakePosition.IN;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeIOReal() {
        wristMotor = new TalonFXAutoLogged(CANIds.INTAKE_WRIST_MOTOR, Canivore);
        rollerMotor = new TalonFXAutoLogged(CANIds.INTAKE_ROLLER_MOTOR, Canivore);
        cancoder = new CANcoder(CANIds.INTAKE_CANCODER, Canivore);

        cancoderPosition = cancoder.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, cancoderPosition);
        ParentDevice.optimizeBusUtilizationForAll(cancoder);

        BaseStatusSignal[] wSignals = wristMotor.getSignals();
        BaseStatusSignal[] rSignals = rollerMotor.getSignals();
        allIntakeSignals = new BaseStatusSignal[wSignals.length + rSignals.length + 1];
        System.arraycopy(wSignals, 0, allIntakeSignals, 0, wSignals.length);
        System.arraycopy(rSignals, 0, allIntakeSignals, wSignals.length, rSignals.length);
        allIntakeSignals[allIntakeSignals.length - 1] = cancoderPosition;

        wristMotorConfig = new TalonFXConfiguration();
        rollerMotorConfig = new TalonFXConfiguration();

        wristMotorConfig.CurrentLimits.StatorCurrentLimit = 45;
        wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        wristMotorConfig.Feedback.FeedbackRemoteSensorID = CANIds.INTAKE_CANCODER;
        wristMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        // when in tolerence no pid
        wristMotorConfig.ClosedLoopGeneral.GainSchedErrorThreshold = 0.01;
        wristMotorConfig.Slot0.GainSchedBehavior = GainSchedBehaviorValue.Inactive; 

        var pidConfig = wristMotorConfig.Slot0;
        pidConfig.kP = 18.000;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;

        // friction
        pidConfig.kS = 0.320;
        pidConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        // gravity
        pidConfig.kG = 0.3750;
        pidConfig.GravityType = GravityTypeValue.Arm_Cosine;
        pidConfig.GravityArmPositionOffset = 0.131;
        
        rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        wristMotor.getConfigurator().apply(wristMotorConfig);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
    }

    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(allIntakeSignals);
        wristMotor.updateInputsFromRefreshedSignals();
        rollerMotor.updateInputsFromRefreshedSignals();

        inputs.currentCancoderPosition = cancoderPosition.getValueAsDouble();
        inputs.tagertPosition = targetPosition;

        Logger.processInputs("Intake", inputs);
        Logger.processInputs("Intake/Wrist Motor", wristMotor.getInputs());
        Logger.processInputs("Intake/Roller Motor", rollerMotor.getInputs());
    }

    @Override
    public IntakeIOInputsAutoLogged getInputs() {
        return inputs;
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.INTAKE_WRIST_MOTOR:
                return wristMotor.getInputs();
            case CANIds.INTAKE_ROLLER_MOTOR:
                return rollerMotor.getInputs();
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
