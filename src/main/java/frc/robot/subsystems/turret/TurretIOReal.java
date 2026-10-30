package frc.robot.subsystems.turret;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class TurretIOReal implements TurretIO {

    private final TalonFXAutoLogged turnMotor;
    private final TalonFXConfiguration turnMotorConfig;
    private final DigitalInput limitSwitch;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private boolean canMakeItToTarget = true;
    private Rotation2d targetRotation = Rotation2d.kZero;

    public TurretIOReal() {
        turnMotor = new TalonFXAutoLogged(CANIds.TURRET_TURN_MOTOR, Canivore);
        limitSwitch = new DigitalInput(DIO_IDS.TURRET_LIMIT_SWITCH);

        turnMotorConfig = new TalonFXConfiguration();

        var motorOutputConfigs = turnMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        // pid control of the falcon through CTRE's motor configs
        var pidConfig = turnMotorConfig.Slot0;
        pidConfig.kP = 2.250;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;

        pidConfig.kS = 0.245;
        pidConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        turnMotorConfig.ClosedLoopGeneral.GainSchedErrorThreshold = 0.002 * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;

        turnMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        turnMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        turnMotor.getConfigurator().apply(turnMotorConfig);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        Logger.processInputs("Turret/Motor", turnMotor.getInputs());

        inputs.robotRelativeRotation = getRobotRelativeRotation();
        inputs.turretRotation = inputs.robotRelativeRotation.getRotations();
        inputs.absTurretRotation = getRingRotation();
        inputs.targetRotation = targetRotation;

        inputs.canMakeItToTarget = canMakeItToTarget;
        inputs.limitSwitch = !limitSwitch.get();
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs() {
        return turnMotor.getInputs();
    }

    // set command to set the turning motor to a speed -1 to 1
    public void setTurretSpeed(double speed) {
        turnMotor.set(speed);
    }

    @Override
    public void setTurretSetpoint(Rotation2d robotRelativeRotation) {
        this.targetRotation = robotRelativeRotation;
        double diffDeg = MathUtil.inputModulus(
            robotRelativeRotation.getDegrees() - TURRET_CONSTANTS.LIMIT_SWITCH_ROBOT_ROTATION.getDegrees(),
            0.0, 360.0
        );
        double mechRotations = diffDeg / 360.0;
        double motorSetpoint;
        if (mechRotations <= TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT) {
            canMakeItToTarget = true;
            motorSetpoint = mechRotations * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
        } else {
            canMakeItToTarget = false;
            // Clamp to closest soft limit
            double distToMax = mechRotations - TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT;
            double distToMin = 1.0 - mechRotations;
            if (distToMax < distToMin) {
                motorSetpoint = TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
            } else {
                motorSetpoint = 0.0;
            }
        }
        turnMotor.setControl(positionVoltage.withPosition(motorSetpoint));
    }

    @Override
    public void setTurretSetpoint(double rotations) {
        setTurretSetpoint(Rotation2d.fromRotations(rotations));
    }

    // set the turn motor's internal encoder to 0 at the limit switch
    @Override
    public void setTurretZero() {
        turnMotor.setPosition(0);
    }

    @Override
    public double getRingRotation() {
        return turnMotor.getPosition().getValueAsDouble() / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
    }

    @Override
    public Rotation2d getRobotRelativeRotation() {
        double mechRotations = getRingRotation();
        return TURRET_CONSTANTS.LIMIT_SWITCH_ROBOT_ROTATION.plus(Rotation2d.fromRotations(mechRotations));
    }

    @Override
    public double getRobotRelitiveRotation() {
        return getRobotRelativeRotation().getRotations();
    }

}
