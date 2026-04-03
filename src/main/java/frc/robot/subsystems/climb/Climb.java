package frc.robot.subsystems.climb;

import java.nio.channels.Pipe;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private static Climb instance = null;
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public static Climb getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    instance = new Climb(new ClimbIOReal());
                    break;
                case SIM:
                    instance = new Climb(new ClimbIOSim());
                    break;
                default:
                    instance = new Climb(new ClimbIO() {
                    });
                    break;
            }
        }
        return instance;
    }

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

    public boolean getLimitSwitch() {
        return inputs.limitSwitch;
    }

    public void setPositionToZero() {
        io.setPosition(0);
    }

    public void setClimbSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setClimbTargetPosition(ClimbPosition position) {
        io.setTargetPosition(position);
    }

    public static enum ClimbPosition {
        UP(115),
        DOWN(0);

        public final double value;

        private ClimbPosition(double value) {
            this.value = value;
        }
    }

}
