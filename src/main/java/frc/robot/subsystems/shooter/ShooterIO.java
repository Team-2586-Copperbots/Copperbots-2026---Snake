package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default TalonFXInputsAutoLogged getMotorInputs(int id) {
        return null;
    }

    public default void runVoltage(double voltage) {
    }

    public default void setMotorSetpoint(double velovity) {
    }

    public default void setPercentageSpeed(double speed) {
    }
}
