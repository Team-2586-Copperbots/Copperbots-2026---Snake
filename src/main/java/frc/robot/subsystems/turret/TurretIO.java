package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public Rotation2d ringPositionSetpoint = Rotation2d.kZero;
        public double ringSpeedSetpoint = 0.0;

        public Rotation2d currentRingPose = Rotation2d.kZero;
        public double currentRingSpeed = 0.0;
        public boolean isClosedLoop = true;
        public boolean limitSwitch = false;
        public boolean isAtPosition = true;
        public Rotation2d rotationRelitiveToRobotZero = Rotation2d.kZero;
    }

    public default void updateInputs(TurretIOInputs inputs) {
    }

    public default void setTurretSetpoint(Rotation2d rotation) {
    }

    public default void setTurretSpeed(double speed) {
    }

    public default void setTurretZero() {
    }

    public default Rotation2d getRingRotation() {return new Rotation2d();}

    public default Rotation2d getRobotRelitiveRotation() {return new Rotation2d();}

}