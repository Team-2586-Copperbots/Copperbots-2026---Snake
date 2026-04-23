package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GainSchedBehaviorValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.driveUtils.PhoenixUtil.TalonFXMotorControllerSim;

public class TurretIOReal extends TurretIO {
    private TalonFXConfiguration config;

    public TurretIOReal() {
        super();
        config = new TalonFXConfiguration();
        var motorOutputConfigs = config.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        // pid control of the falcon through CTRE's motor configs
        var pidConfig = config.Slot0;
        pidConfig.kP = 2.250;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;

        pidConfig.kS = 0.245;
        pidConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.ClosedLoopGeneral.GainSchedErrorThreshold = 0.002 * TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
        config.Slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput;

        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        turnMotor.getConfigurator().apply(config);

    }

    @Override
    public void periodic() {
        Logger.processInputs("Turret/Motor", motorInputs.getInputs(turnMotor));

        inputs.turretRotation = getRobotRelitiveRotation();
        inputs.absTurretRotation = getRingRotation();

        inputs.canMakeItToTarget = canMakeItToTarget;
        inputs.limitSwitch = !limitSwitch.get();
    }

}
