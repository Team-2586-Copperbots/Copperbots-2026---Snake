package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class Intake extends SubsystemBase {
    private final TalonFX movementBarMotor;
    private final TalonFX spinnerMotor;
    private final TalonFXConfiguration movementBarMotorConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration spinnerMotorConfig = new TalonFXConfiguration();
    private ProfiledPIDController PIDie;
    private Constraints PIDConstraints;

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

    public Intake() {
        movementBarMotor = new TalonFX(CANIds.IntakeMovementBarMotorCANID);
        spinnerMotor = new TalonFX(CANIds.IntakeSpinnerMotorCANID);

        movementBarMotor.getConfigurator().apply(movementBarMotorConfig);
        spinnerMotor.getConfigurator().apply(spinnerMotorConfig);

        PIDConstraints = new Constraints(300, 400);
        PIDie = new ProfiledPIDController(.05, 0.05, 0, PIDConstraints);
    }

    public void setMovementBarSpeed(double speed) {
        movementBarMotor.set(speed);
    }

    public void setSpinnerSpeed(double speed) {
        spinnerMotor.set(speed);
    }

    public double getMovementBarSpeed() {
        return movementBarMotor.getPosition().getValueAsDouble();
    }
}
