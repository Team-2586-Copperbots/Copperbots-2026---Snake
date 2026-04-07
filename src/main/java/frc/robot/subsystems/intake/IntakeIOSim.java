package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.intake.Intake.IntakePosition;

import static edu.wpi.first.units.Units.Meter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class IntakeIOSim implements IntakeIO {

    @SuppressWarnings("unused")
    private IntakePosition targetPosition = IntakePosition.IN;
    public static IntakeSimulation simulatedIntake = null;
    private Distance width = Meter.of(Units.inchesToMeters(23));
    private Distance length = Meter.of(Units.inchesToMeters(9.5));

    public IntakeIOSim(SwerveDriveSimulation drive) {
        simulatedIntake = IntakeSimulation.OverTheBumperIntake("Fule", drive, width,
                length, IntakeSide.FRONT, 80);
    }

    @Override
    public void updateInputs() {
        // inputs.rollerSetpoint = rollerSpeedSetpoint;
        // inputs.wristSetpoint = targetPosition;
        // inputs.wristIsClosedLoop = closedLoop;

        // inputs.currentWristPosition = targetPosition.value;
        // inputs.currentCancoderPosition = targetPosition.value / INTAKE_CONSTANTS.rotorToSensor;
        // inputs.currentRollerSpeed = rollerSpeedSetpoint;
    }

    @Override
    public void setRollerSpeed(double speed) {
    }

    @Override
    public void setWristPositionTarget(IntakePosition position) {
        targetPosition = position;
        if (position.equals(IntakePosition.OUT)) {
            simulatedIntake.startIntake();
        } else {
            simulatedIntake.stopIntake();
        }
    }

    @Override
    public void setWristSpeed(double speed) {
    }
}
