package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.util.auto_logging_stuff.LoggedTalonFXInputs;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

import static frc.robot.Constants.CANIds.Canivore;

public class ShooterIO {

    // motors
    protected final TalonFX shooterMotor1, shooterMotor2;
    protected final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

    protected final LoggedTalonFXInputs motor1Inputs, motor2Inputs;

    public ShooterIO() {

        shooterMotor1 = new TalonFX(CANIds.SHOOTER_MOTOR_1, Canivore);
        shooterMotor2 = new TalonFX(CANIds.SHOOTER_MOTOR_2, Canivore);

        motor1Inputs = new LoggedTalonFXInputs();
        motor2Inputs = new LoggedTalonFXInputs();

    }

    public void periodic() {
    }

    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        return null;
    }

    public void runVoltage(double voltage) {
        shooterMotor1.setVoltage(voltage);
    }

    public void setMotorSetpoint(double velocity) {
        Logger.recordOutput("commanded velocity", velocity);
        shooterMotor1.setControl(velocityVoltage.withVelocity(velocity));
    }

    public void setPercentageSpeed(double speed) {
        shooterMotor1.set(speed);
    }
}
