package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.Intake.IntakePosition;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public boolean motorIsOK = false;
        public double motorVolts = 0;
        public double motorAmps = 0;
        public double motorSetpoint = 0;
        public double motorRotation = 0;
        public boolean motorIsClosedLoop = true;



        public boolean limitSwitch = false;
        public boolean canMakeItToPosition = true;
    }

    public default void updateInputs(TurretIOInputs inputs) {
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