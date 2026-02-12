package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.CANIds.Canivore;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.Constants.TURRET_CONSTANTS;

//
//
// this subsystem works in rotations!!!
//
//

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turnMotor;
    private final TalonFXConfiguration turnMotorConfig;
    private final DigitalInput limitSwitch;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    public TurretSubsystem() {
        turnMotor = new TalonFX(CANIds.TURRET_TURN_MOTOR, Canivore);
        limitSwitch = new DigitalInput(DIO_IDS.TURRET_LIMIT_SWITCH);

        turnMotorConfig = new TalonFXConfiguration();

        // turnMotorConfig.Feedback.FeedbackRemoteSensorID = CANcoder.getDeviceID();
        // turnMotorConfig.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.RemoteCANcoder;
        // turnMotorConfig.Feedback.RotorToSensorRatio = 11.2;

        var motorOutputConfigs = turnMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        // pid control of the falcon through CTRE's motor configs
        var pidConfig = turnMotorConfig.Slot0;
        pidConfig.kP = 1.500;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;
        pidConfig.kS = 0.050;
        // pidConfig.kV = 8.000;

        turnMotor.getConfigurator().apply(turnMotorConfig);

    }

    // set comand to set the turning motor to a speed -1 to 1
    public void setTurnMotorSpeed(double speed) {
        turnMotor.set(speed);
    }

    // set the turn motors's internal encoder
    public void setTurnMotorPosition(double rotation) {
        turnMotor.setPosition(rotation);
    }

    // this uses the CTRE motors built-in positionVoltage controler to set the angle
    // of the turret within the limits of 0-320 degreas (commented out)
    public void setTurretRotation(double roations) {

        // limits are typed in as degres

        if (roations >= (0 - TURRET_CONSTANTS.TURRET_ZERO_TO_ROBOT_ZERO_OFFSET)
                && roations < TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT) {
            turnMotor.setControl(
                    positionVoltage.withPosition((roations + TURRET_CONSTANTS.TURRET_ZERO_TO_ROBOT_ZERO_OFFSET)
                            * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO));
        } else {
            double startOfEndZone = 0;
            double endOfEndZone = TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT;

            if (Math.abs((startOfEndZone - roations)) < Math.abs((endOfEndZone - roations))) {
                turnMotor.setControl(positionVoltage.withPosition(
                        0 * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO));
            } else if (Math.abs((startOfEndZone - roations)) >= Math.abs((endOfEndZone - roations))) {
                turnMotor.setControl(positionVoltage.withPosition(
                        TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO));
            } else {
                turnMotor.setControl(positionVoltage.withPosition(
                        0 * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO));
            }

        }
    }

    // command to aim at the hub
    // drivetrain is passed to the calculating method
    public void aimAtHub(CommandSwerveDrivetrain drivetrain) {
        // aims at hub
        setTurretRotation(Utils.getAngleToHub(drivetrain));
    }

    public double getRingRotation() {
        return turnMotor.getPosition().getValueAsDouble() / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
    }

    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ring rotation", getRingRotation());
        SmartDashboard.putString("positionVoltage", positionVoltage.getPositionMeasure().toLongString());
        SmartDashboard.putBoolean("limit switch", getLimitSwitch());

    }
}
