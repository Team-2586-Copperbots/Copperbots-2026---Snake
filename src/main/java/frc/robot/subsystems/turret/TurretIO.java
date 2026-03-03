package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public boolean limitSwitch = false;
        public boolean isClosedLoop = true;
        public double ringPositionSetpoint = 0.0;
        public double ringSpeedSetpoint = 0.0;
        public double currentRingPose = 0.0;
        public double currentRingSpeed = 0.0;
        public boolean isAtPosition = true;
        public double rotationRelitiveToRobotZero = 0;
    }

    public default void updateInputs(TurretIOInputs inputs) {
    }

    public default void setTurretSetpoint(double rotation) {
    }

    public default void setTurretSpeed(double speed) {
    }

    public default void setTurretZero() {
    }

    public default double getRingRotation() {return 0;}

    public default double getRobotRelitiveRotation() {return 0;}

}