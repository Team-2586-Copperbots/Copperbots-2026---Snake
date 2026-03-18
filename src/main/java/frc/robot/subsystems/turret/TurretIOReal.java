package frc.robot.subsystems.turret;

import static frc.robot.Constants.CANIds.Canivore;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.Constants.TURRET_CONSTANTS;

public class TurretIOReal implements TurretIO {

    // every thing workis in rotations!
    // everything is messured from the limit switch

    private final TalonFX turnMotor;
    private final TalonFXConfiguration turnMotorConfig;
    private final DigitalInput limitSwitch;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private boolean isClosedLoop = true;
    private boolean isAtPosition = true;

    public TurretIOReal() {
        turnMotor = new TalonFX(CANIds.TURRET_TURN_MOTOR, Canivore);
        limitSwitch = new DigitalInput(DIO_IDS.TURRET_LIMIT_SWITCH);

        turnMotorConfig = new TalonFXConfiguration();

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

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.isClosedLoop = isClosedLoop;
        inputs.currentRingPose = getRingRotation();
        inputs.currentRingSpeed = turnMotor.getVelocity().getValueAsDouble();
        inputs.limitSwitch = !limitSwitch.get();
        inputs.isAtPosition = isAtPosition;

        inputs.rotationRelitiveToRobotZero = getRobotRelitiveRotation();
        inputs.ringPositionSetpoint = positionVoltage.Position;
    }

    // set comand to set the turning motor to a speed -1 to 1
    public void setTurretSpeed(double speed) {
        isClosedLoop = false;
        turnMotor.set(speed);
    }

    // this uses the CTRE motors built-in positionVoltage controler to set the angle
    // of the turret within the limits of 0-320 degreas
    @Override
    public void setTurretSetpoint(double roations) {
        isClosedLoop = true;

        if ((roations >= 0)
                && (roations < TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT)) {
            isAtPosition = true;
            turnMotor.setControl(
                    positionVoltage.withPosition(((roations + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET)
                            * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO)));
        } else {
            isAtPosition = false;
            turnMotor.setControl(positionVoltage.withPosition(1));
        }

    }

    // set the turn motors's internal encoder
    @Override
    public void setTurretZero() {
        turnMotor.setPosition(0);
    }

    @Override
    public double getRingRotation() {
        return turnMotor.getPosition().getValueAsDouble() / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
    }

    @Override
    public double getRobotRelitiveRotation() {
        return getRingRotation() + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET;
    }

}
