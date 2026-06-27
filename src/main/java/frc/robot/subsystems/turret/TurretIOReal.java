package frc.robot.subsystems.turret;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GainSchedKpBehaviorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.auto_logging_stuff.TalonFXAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class TurretIOReal implements TurretIO {

    // every thing workis in rotations!
    // everything is messured from the limit switch

    private final TalonFXAutoLogged turnMotor;
    private final TalonFXConfiguration turnMotorConfig;
    private final DigitalInput limitSwitch;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private boolean canMakeItToTarget = true;

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
        turnMotorConfig.Slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;

        turnMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        turnMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;

        turnMotor.getConfigurator().apply(turnMotorConfig);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        Logger.processInputs("Turret/Motor", turnMotor.getInputs());

        inputs.turretRotation = getRobotRelitiveRotation();
        inputs.absTurretRotation = getRingRotation();

        inputs.canMakeItToTarget = canMakeItToTarget;
        inputs.limitSwitch = !limitSwitch.get();
    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs() {
        return turnMotor.getInputs();
    }

    // set comand to set the turning motor to a speed -1 to 1
    public void setTurretSpeed(double speed) {
        turnMotor.set(speed);
    }

    // this uses the CTRE motors built-in positionVoltage controler to set the angle
    // of the turret within the limits of 0-320 degreas
    @Override
    public void setTurretSetpoint(double roations) {
        if ((roations >= 0)
                && (roations < TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT)) {
            canMakeItToTarget = true;
            turnMotor.setControl(
                    positionVoltage.withPosition(((roations + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET)
                            * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO)));
        } else {
            canMakeItToTarget = false;
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
