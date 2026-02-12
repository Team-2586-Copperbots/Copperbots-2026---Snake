package frc.robot.subsystems;

import static frc.robot.Constants.CANIds.Canivore;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.IntakePosition;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX movementMotor;
    private final TalonFX spinnerMotor;
    private final CANcoder cancoder;
    private final TalonFXConfiguration movementMotorConfig;
    private final TalonFXConfiguration spinnerMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    public IntakeSubsystem() {
        movementMotor = new TalonFX(CANIds.INTAKE_MOVEMENT_MOTOR_ID, Canivore);
        spinnerMotor = new TalonFX(CANIds.INTAKE_SPINNER_MOTOR_ID, Canivore);
        cancoder = new CANcoder(CANIds.INTAKE_CANCODER, Canivore);

        movementMotorConfig = new TalonFXConfiguration();
        // movementMotorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        // movementMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // movementMotorConfig.Feedback.RotorToSensorRatio = 100;
        // movementMotorConfig.Feedback.SensorToMechanismRatio = 1;

        spinnerMotorConfig = new TalonFXConfiguration();

        // var motorOutputConfigs = movementMotorConfig.MotorOutput;
        // motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        // var pidConfig = movementMotorConfig.Slot0;
        // // TODO: tune pid at all!?
        // pidConfig.kP = 0.05;
        // pidConfig.kI = 0.0;
        // pidConfig.kD = 0.0025;

        movementMotor.getConfigurator().apply(movementMotorConfig);
        spinnerMotor.getConfigurator().apply(spinnerMotorConfig);
    }

    // Constants.IntakePosition
    public void setIntakePosition(IntakePosition position) {
        movementMotor.setControl(positionVoltage.withPosition(position.getValue()));
    }

    // positive is out
    public void setMovementBarSpeed(double speed) {
        movementMotor.set(speed);
    }

    public void setSpinnerSpeed(double speed) {
        spinnerMotor.set(speed);
    }

    public double getMovementBarPosition() {
        return movementMotor.getPosition().getValueAsDouble();
    }

    public Command setMovementMotorSpeedCommand(double speed) {
        return runEnd(() -> {
            setMovementBarSpeed(speed);
        }, () -> {
            setMovementBarSpeed(0);
        });
    }

    public Command setSpinMotorSpeedCommand(double speed) {
        return runEnd(() -> {
            setSpinnerSpeed(speed);
        }, () -> {
            setSpinnerSpeed(0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake position in rotations", getMovementBarPosition());
    }
}
