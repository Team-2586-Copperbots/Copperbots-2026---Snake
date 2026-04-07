package frc.robot.subsystems.shooter;

public class ShooterIOSim implements ShooterIO {
    @SuppressWarnings("unused")
    private double velocityVoltagePlaceholder;

    public ShooterIOSim() {
        velocityVoltagePlaceholder = 0;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
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
