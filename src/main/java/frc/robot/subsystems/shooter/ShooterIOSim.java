package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {
    private double velocityVoltagePlaceholder;

    public ShooterIOSim() {
        velocityVoltagePlaceholder = 0;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorSetpoint = velocityVoltagePlaceholder;
        inputs.currentMotorSpeed = velocityVoltagePlaceholder;
    }

    @Override
    public void setMotorSetpoint(double velocity) {
        velocityVoltagePlaceholder = velocity;
    }

    @Override
    public void setPercentageSpeed(double speed) {
        velocityVoltagePlaceholder = speed * 100;
    }
}
