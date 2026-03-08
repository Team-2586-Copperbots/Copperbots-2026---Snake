package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double ringPositionSetpoint = 0;
        public double ringSpeedSetpoint = 0.0;

        public double currentRingPose = 0;
        public double currentRingSpeed = 0.0;
        public boolean isClosedLoop = false;
        public boolean limitSwitch = false;
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