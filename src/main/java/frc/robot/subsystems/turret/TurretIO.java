package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double absTurretRotation = 0;
        public double turretRotation = 0;

        public boolean limitSwitch = false;
        public boolean canMakeItToTarget = false;
    }

    public default void updateInputs() {
    }

    public default TalonFXInputsAutoLogged getMotorInputs() {
        return null;
    }

    public default void setTurretSetpoint(double rotation) {
    }

    public default void setTurretSpeed(double speed) {
    }

    public default void setTurretZero() {
    }

    public default double getRingRotation() {
        return 0;
    }

    public default double getRobotRelitiveRotation() {
        return 0;
    }

}