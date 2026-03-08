package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.intake.Intake.IntakePosition;

import static edu.wpi.first.units.Units.Meter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class IntakeIOSim implements IntakeIO {

    private IntakePosition targetPosition = IntakePosition.IN;
    private IntakeSimulation simulatedIntake;
    private Distance width = Distance.ofBaseUnits(edu.wpi.first.math.util.Units.inchesToMeters(23), Meter);
    private Distance length = Distance.ofBaseUnits(edu.wpi.first.math.util.Units.inchesToMeters(9.5), Meter);

    public IntakeIOSim(SwerveDriveSimulation drive) {
        simulatedIntake = IntakeSimulation.OverTheBumperIntake("Fule", drive, width,
                length, IntakeSide.FRONT, 80);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

        inputs.wristSetpoint = targetPosition;
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
