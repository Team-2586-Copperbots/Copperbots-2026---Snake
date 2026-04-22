package frc.robot.subsystems.turret;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.GainSchedKpBehaviorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.auto_logging_stuff.LoggedTalonFXInputs;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

import org.littletonrobotics.junction.AutoLog;

public class TurretIO {
    
    @AutoLog
    public static class TurretIOInputs {
        public double absTurretRotation = 0;
        public double turretRotation = 0;

        public boolean limitSwitch = false;
        public boolean canMakeItToTarget = false;
    }
    
    // implements TurretIO
    // every thing workis in rotations!
    // everything is messured from the limit switch

    protected final TalonFX turnMotor;
    protected final TalonFXConfiguration turnMotorConfig;
    protected final DigitalInput limitSwitch;
    protected final PositionVoltage positionVoltage = new PositionVoltage(0);
    protected boolean canMakeItToTarget = false;

    protected TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    protected LoggedTalonFXInputs motorInputs = new LoggedTalonFXInputs();

    public TurretIO() {
        turnMotor = new TalonFX(CANIds.TURRET_TURN_MOTOR, Canivore);
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

    public void updateInputs() {
        Logger.processInputs("Turret/Motor", motorInputs.getInputs(turnMotor));

        inputs.turretRotation = getRobotRelitiveRotation();
        inputs.absTurretRotation = getRingRotation();

        inputs.canMakeItToTarget = canMakeItToTarget;
        inputs.limitSwitch = !limitSwitch.get();
    }

    public TalonFXInputsAutoLogged getMotorInputs() {
        return motorInputs.getInputs(turnMotor);
    }

    // set comand to set the turning motor to a speed -1 to 1
    public void setTurretSpeed(double speed) {
        turnMotor.set(speed);
    }

    // this uses the CTRE motors built-in positionVoltage controler to set the angle
    // of the turret within the limits of 0-320 degreas
    public void setTurretSetpoint(double roations) {
        if ((roations >= 0)
                && (roations < TURRET_CONSTANTS.ROTATION_RANGE_IN_ROT)) {
            canMakeItToTarget = true;
            turnMotor.setControl(
                    positionVoltage
                            .withPosition(((roations + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET)
                                    * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO)));
        } else {
            canMakeItToTarget = false;
            turnMotor.setControl(positionVoltage.withPosition(1));
        }

    }

    // set the turn motors's internal encoder
    public void setTurretZero() {
        turnMotor.setPosition(0);
    }

    public double getRingRotation() {
        return turnMotor.getPosition().getValueAsDouble() / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
    }

    public double getRobotRelitiveRotation() {
        return getRingRotation() + TURRET_CONSTANTS.TURRET_RING_MINIMUM_TO_ROBOT_BACK_OFFSET;
    }

}