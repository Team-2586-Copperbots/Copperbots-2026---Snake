package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turnMotor;
    private final TalonFX spinnerMotor;
    private final TalonFXConfiguration turnMotorConfig;
    private final TalonFXConfiguration spinnerMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    public TurretSubsystem() {
        turnMotor = new TalonFX(CANIds.TURRET_TURN_MOTOR);
        spinnerMotor = new TalonFX(CANIds.TURRET_SPIN_MOTOR);

        turnMotorConfig = new TalonFXConfiguration();
        spinnerMotorConfig = new TalonFXConfiguration();

        var motorOutputConfigs = turnMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = motorOutputConfigs.NeutralMode.Coast;

        var pidConfig = turnMotorConfig.Slot0;
        // TODO: tune pid
        pidConfig.kP = 0.0;
        pidConfig.kI = 0.0;
        pidConfig.kD = 0.0;

        turnMotor.getConfigurator().apply(turnMotorConfig);
        spinnerMotor.getConfigurator().apply(spinnerMotorConfig);
    }

    public Command setintakePosition(Double angle) {
        if (angle > 0 && angle < 320) {
            return runOnce(() -> turnMotor.setControl(positionVoltage.withPosition(angle/360)));
        } else {
            return runOnce(() -> turnMotor.setControl(positionVoltage.withPosition(0/360)));
        }
    }

    public void setMovementBarSpeed(double speed) {
        turnMotor.set(speed);
    }

    public void setSpinnerSpeed(double speed) {
        spinnerMotor.set(speed);
    }

    public double getMovementBarSpeed() {
        return turnMotor.getPosition().getValueAsDouble();
    }
}
