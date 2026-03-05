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

    public void setClimbSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setClimbPosition(ClimbPosition position) {
        io.setPosition(position);
    }

    public static enum ClimbPosition {
        UP(10),
        DOWN(0);

        private final double position;

        private ClimbPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

}
