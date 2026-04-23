package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.GeneralUtils;

public class TurretIOSim extends TurretIO {
    // private MapleMotorSim motorSim2;
    // private GenericMotorController conroler;

    private TalonFXSimState sim;
    private final TalonFXConfiguration turnMotorConfig;
    private DCMotorSim motorSim;
    private double moi = 0.0512119394 * 0.0001;
    

    public TurretIOSim() {
        super();
        
    Logger.recordOutput("things", moi);
        turnMotorConfig = new TalonFXConfiguration();
        var motorOutputConfigs = turnMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        // pid control of the falcon through CTRE's motor configs
        var pidConfig = turnMotorConfig.Slot0;
        pidConfig.kP = 1.500;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.200;

        turnMotor.getConfigurator().apply(turnMotorConfig);

        // motorSim2 = new MapleMotorSim(new SimMotorConfigs(DCMotor.getFalcon500(1),
        // TURRET_CONSTANTS.MOTOR_TO_RING_RATIO,
        // KilogramSquareMeters.of(moi), Volts.of(0.1)));
        // conroler = new
        // SimulatedMotorController.GenericMotorController(DCMotor.getFalcon500(1));

        sim = turnMotor.getSimState();
        motorSim = new DCMotorSim(
                LinearSystemId.createSingleJointedArmSystem(DCMotor.getFalcon500(1), moi,
                        GeneralUtils.invert(TURRET_CONSTANTS.MOTOR_TO_RING_RATIO)),
                DCMotor.getFalcon500(1), 0, 0);
    }

    @Override
    public void periodic() {
        sim.setSupplyVoltage(Volts.of(12.0));
        motorSim.setInputVoltage(sim.getMotorVoltage());

        motorSim.update(0.02);

        sim.setRawRotorPosition(motorSim.getAngularPosition());
        sim.setRotorVelocity(motorSim.getAngularVelocity());

        inputs.turretRotation = getRobotRelitiveRotation();
        inputs.absTurretRotation = getRingRotation();

        inputs.canMakeItToTarget = canMakeItToTarget;
        inputs.limitSwitch = true;

        motorInputs.log("Turret/Motor", sim, motorSim, turnMotor);
        Logger.processInputs("Turret", inputs);

    }

    @Override
    public double getRingRotation() {
        return motorSim.getAngularPositionRotations() / TURRET_CONSTANTS.MOTOR_TO_RING_RATIO;
    }

    @Override
    public void setTurretZero() {
        motorSim.setAngle(Rotations.of(0).in(Radians));
    }

}
