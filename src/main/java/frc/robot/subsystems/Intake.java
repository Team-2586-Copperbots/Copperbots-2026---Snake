package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Intake extends SubsystemBase {
    private final TalonFX movementMotor;
    private final TalonFX spinnerMotor;
    private final TalonFXConfiguration movementMotorConfig;
    private final TalonFXConfiguration spinnerMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    public IntakePosition currentPosition = IntakePosition.in;
    public IntakePosition targetPosition = IntakePosition.out;

    public enum IntakePosition {
        in(2),
        out(40),
        halfWay(12);

        private final int value;

        private IntakePosition(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    @SuppressWarnings("static-access")
    public Intake() {
        movementMotor = new TalonFX(CANIds.INTAKE_MOVEMENT_MOTOR_ID);
        spinnerMotor = new TalonFX(CANIds.INTAKE_SPINNER_MOTOR_ID);

        movementMotorConfig = new TalonFXConfiguration();
        spinnerMotorConfig = new TalonFXConfiguration();

        var motorOutputConfigs = movementMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = motorOutputConfigs.NeutralMode.Coast;

        var pidConfig = movementMotorConfig.Slot0;
        // TODO: tune pid better?
        pidConfig.kP = 0.28;
        pidConfig.kI = 0.0;
        pidConfig.kD = 0.0075;

        movementMotor.getConfigurator().apply(movementMotorConfig);
        spinnerMotor.getConfigurator().apply(spinnerMotorConfig);
    }

    public Command setintakePosition(IntakePosition position) {
        return runOnce(() -> movementMotor.setControl(positionVoltage.withPosition(position.getValue())));
    }

    public void setMovementBarSpeed(double speed) {
        movementMotor.set(speed);
    }

    public void setSpinnerSpeed(double speed) {
        spinnerMotor.set(speed);
    }

    public double getMovementBarSpeed() {
        return movementMotor.getPosition().getValueAsDouble();
    }
}
