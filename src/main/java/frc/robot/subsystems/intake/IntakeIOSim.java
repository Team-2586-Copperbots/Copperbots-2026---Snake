package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.INTAKE_CONSTANTS;
import frc.robot.subsystems.intake.Intake.IntakePosition;

import static edu.wpi.first.units.Units.Meter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class IntakeIOSim implements IntakeIO {

    private boolean closedLoop = true;
    private IntakePosition targetPosition = IntakePosition.IN;
    private double wristSpeedSetpoint = 0;
    private double rollerSpeedSetpoint = 0;
    private IntakeSimulation simulatedIntake;
    private Distance width = Distance.ofBaseUnits(edu.wpi.first.math.util.Units.inchesToMeters(23), Meter);
    private Distance length = Distance.ofBaseUnits(edu.wpi.first.math.util.Units.inchesToMeters(9.5), Meter);

    public IntakeIOSim(SwerveDriveSimulation drive) {
        simulatedIntake = IntakeSimulation.OverTheBumperIntake("Fule", drive, width,
                length, IntakeSide.FRONT, 80);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerSetpoint = rollerSpeedSetpoint;
        inputs.wristSetpoint = targetPosition;
        inputs.percentageWristSpeed = wristSpeedSetpoint;
        inputs.isClosedLoop = closedLoop;

        inputs.currentWristPosition = targetPosition.value;
        inputs.currentCancoderPosition = targetPosition.value / INTAKE_CONSTANTS.rotorToSensor;
        inputs.currentRollerSpeed = rollerSpeedSetpoint;
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerSpeedSetpoint = speed;
    }

    @Override
    public void setWristPositionTarget(IntakePosition position) {
        closedLoop = true;
        targetPosition = position;
        if (position.equals(IntakePosition.OUT)) {
            simulatedIntake.startIntake();
        } else {
            simulatedIntake.stopIntake();
        }
    }

    @Override
    public void setWristSpeed(double speed) {
        closedLoop = false;
        wristSpeedSetpoint = speed;
    }
}
