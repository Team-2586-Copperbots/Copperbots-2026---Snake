package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakePosition;

import static edu.wpi.first.units.Units.Meter;
import static frc.robot.Constants.CANIds.Canivore;

import org.dyn4j.UnitConversion;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.gamepieces.GamePiece;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

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
    public void setWristPosition(IntakePosition position) {
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
