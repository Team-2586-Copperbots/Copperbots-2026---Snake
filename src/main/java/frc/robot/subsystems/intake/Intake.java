package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.INTAKE_CONSTANTS;
import frc.robot.subsystems.drive.Drive;

public class Intake extends SubsystemBase {
    private static Intake instance = null;
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public Intake() {
        switch (Constants.currentMode) {
            case REAL:
                io = new IntakeIOReal();
                break;

            case SIM:
                io = new IntakeIOSim(Drive.getInstance().driveSimulation);
                break;

            default:
                io = new IntakeIO() {
                };
                break;
        }
    }

    // Constants.IntakePosition
    public void setIntakePositionTarget(IntakePosition position) {
        io.setWristPositionTarget(position);
    }

    // positive is out
    public void setWristSpeed(double speed) {
        io.setWristSpeed(speed);
    }

    public void setRollerSpeed(double speed) {
        if ((Math.abs(io.getMotorInputs(CANIds.INTAKE_WRIST_MOTOR).position
                - IntakePosition.IN.value)) < INTAKE_CONSTANTS.distanceToStopAt) {
            io.setRollerSpeed(0);
        } else {
            io.setRollerSpeed(speed);
        }
    }

    public double getWristPosition() {
        return io.getMotorInputs(CANIds.INTAKE_WRIST_MOTOR).position;
    }

    public IntakePosition getWristTarget() {
        return inputs.tagertPosition;
    }

    public boolean getIsDown() {
        if (((getWristPosition() - IntakePosition.IN.value) < INTAKE_CONSTANTS.idDownThreshold)
                || ((getWristPosition() - IntakePosition.OUT.value) < INTAKE_CONSTANTS.idDownThreshold)) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        io.updateInputs();
    }

    public static enum IntakePosition {
        IN(0.0),
        OUT(0.251),
        part(0.2);
        // ,
        // HALFWAY(4);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }
}
