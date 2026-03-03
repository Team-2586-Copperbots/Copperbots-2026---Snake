package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double currentMotorSpeed = 0;
        public double[] motorCurents = new double[2];
        public double motorSetpoint = 10;
        public double percentageSpeed = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setMotorSetpoint(double velovity) {
    }

    public default void setPercentageSpeed(double speed) {
    }
}
