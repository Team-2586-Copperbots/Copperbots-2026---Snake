package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX movementMotor;
    private final TalonFX spinnerMotor;
    private final TalonFXConfiguration movementMotorConfig;
    private final TalonFXConfiguration spinnerMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    public IntakePosition currentPosition = IntakePosition.IN;
    public IntakePosition targetPosition = IntakePosition.OUT;

    public enum IntakePosition {
        IN(0),
        OUT(40),
        HALFWAY(12);

        private final int value;

        private IntakePosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public IntakeSubsystem() {
        movementMotor = new TalonFX(CANIds.INTAKE_MOVEMENT_MOTOR_ID);
        spinnerMotor = new TalonFX(CANIds.INTAKE_SPINNER_MOTOR_ID);

        movementMotorConfig = new TalonFXConfiguration();
        spinnerMotorConfig = new TalonFXConfiguration();

        var motorOutputConfigs = movementMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        var pidConfig = movementMotorConfig.Slot0;
        // TODO: tune pid at all!?
        pidConfig.kP = 0.05;
        pidConfig.kI = 0.0;
        pidConfig.kD = 0.0025;

        movementMotor.getConfigurator().apply(movementMotorConfig);
        spinnerMotor.getConfigurator().apply(spinnerMotorConfig);
    }

    public void setIntakePosition(IntakePosition position) {
        movementMotor.setControl(positionVoltage.withPosition(position.getValue()));
    }

    public void setMovementBarSpeed(double speed) {
        movementMotor.set(speed);
    }

    public void setSpinnerSpeed(double speed) {
        spinnerMotor.set(speed);
    }

    public double getMovementBarPosition() {
        return movementMotor.getPosition().getValueAsDouble();
    }

    public Command setMovementMotorSpeed(double speed) {
        return runOnce(() -> {
            setMovementBarSpeed(speed);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake position in rotations", getMovementBarPosition());
    }
}
