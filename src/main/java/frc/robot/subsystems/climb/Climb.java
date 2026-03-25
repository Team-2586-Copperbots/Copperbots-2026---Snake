package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public double getPosition() {
        return inputs.motorPosition;
    }

    public void setClimbSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setClimbTargetPosition(ClimbPosition position) {
        io.setTargetPosition(position);
    }

    public static enum ClimbPosition {
        UP(10),
        DOWN(0);

        public final double value;

        private ClimbPosition(double value) {
            this.value = value;
        }
    }

}
