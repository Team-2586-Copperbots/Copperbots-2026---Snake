package frc.robot.subsystems.climb;

import java.nio.channels.Pipe;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
    private static Climb instance = null;
    private ClimbIO io;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    public Climb() {
        switch (Constants.currentMode) {
            case REAL:
                io = new ClimbIOReal();
                break;
            case SIM:
                io = new ClimbIOSim();
                break;
            default:
                io = new ClimbIO() {
                };
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateAndLogInputs();
    }

    public double getPosition() {
        return io.getMotorInputs(1).position;
    }

    public boolean getLimitSwitch() {
        return io.getInputs().limitSwitch;
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
