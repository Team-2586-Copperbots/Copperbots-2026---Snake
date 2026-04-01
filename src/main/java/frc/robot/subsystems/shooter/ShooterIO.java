package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.intake.Intake.IntakePosition;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean[] motorIsOK = new boolean[2];
        public double[] motorVolts = new double[2];
        public double[] motorAmps = new double[2];
        public double[] motorTemps = new double[2];
        public double motorSetpoint = 0;
        public double motorSpeed = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {
    }

    public default void setMotorSetpoint(double velovity) {
    }

    public default void setPercentageSpeed(double speed) {
    }
}
