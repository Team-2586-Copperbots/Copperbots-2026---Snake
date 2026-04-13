package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.util.auto_logging_stuff.SimMotorAutoLogged;

import static edu.wpi.first.units.Units.Meter;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class IntakeIOSim implements IntakeIO {
    private SimMotorAutoLogged wristMotor;
    private SimMotorAutoLogged rollerMotor;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private IntakePosition targetPosition = IntakePosition.IN;
    // public static IntakeSimulation simulatedIntake = null;
    // private Distance width = Meter.of(Units.inchesToMeters(23));
    // private Distance length = Meter.of(Units.inchesToMeters(9.5));

    public IntakeIOSim(SwerveDriveSimulation drive) {
        rollerMotor = new SimMotorAutoLogged();
        wristMotor = new SimMotorAutoLogged();
        // simulatedIntake = IntakeSimulation.OverTheBumperIntake("Fule", drive, width,
        //         length, IntakeSide.FRONT, 80);
    }

    @Override
    public void updateInputs() {
        inputs.currentCancoderPosition = wristMotor.getInputs().position;
        inputs.tagertPosition = targetPosition;
        Logger.processInputs("Intake", inputs);
        Logger.processInputs("Intake/wrist motor", wristMotor.getInputs());
        Logger.processInputs("Intake/roller motor", rollerMotor.getInputs());
    }

    @Override
    public void setRollerSpeed(double speed) {
        rollerMotor.setSimSpeed(speed);
    }

    @Override
    public void setWristPositionTarget(IntakePosition position) {
        targetPosition = position;
        wristMotor.setSimTarget(targetPosition.value, false);
        // if (position.equals(IntakePosition.OUT)) {
        //     simulatedIntake.startIntake();
        // } else {
        //     simulatedIntake.stopIntake();
        // }
    }

    @Override
    public void setWristSpeed(double speed) {
        wristMotor.setSimSpeed(speed);
    }
}
