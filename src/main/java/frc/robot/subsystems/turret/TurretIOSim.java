package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.driveUtils.PhoenixUtil.TalonFXMotorControllerSim;

public class TurretIOSim extends TurretIO {
    // private MapleMotorSim motorSim2;
    // private GenericMotorController conroler;

    private TalonFXSimState sim;
    private DCMotorSim motorSim;
    private double moi = 0.0512119394;
    private MomentOfInertia MoI = KilogramSquareMeters.of(moi);

    public TurretIOSim() {
        super();
        // motorSim2 = new MapleMotorSim(new SimMotorConfigs(DCMotor.getFalcon500(1), TURRET_CONSTANTS.MOTOR_TO_RING_RATIO,
        //         MoI, Volts.of(0.01)));
        // conroler = new SimulatedMotorController.GenericMotorController(DCMotor.getFalcon500(1));

        sim = turnMotor.getSimState();
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), moi, TURRET_CONSTANTS.MOTOR_TO_RING_RATIO),
                DCMotor.getFalcon500(1), null);
    }

    @Override
    public void updateInputs() {
        // SimulatedBattery.addMotor(null);
        sim.setSupplyVoltage(12);
        motorSim.setInputVoltage(sim.getMotorVoltage());

        motorSim.update(0.02);

        sim.setRawRotorPosition(motorSim.getAngularPosition());
        sim.setRotorVelocity(motorSim.getAngularVelocity());
        sim.setRotorAcceleration(motorSim.getAngularAcceleration());

        Logger.processInputs("Turret/Motor", motorInputs.getInputs(turnMotor));

        inputs.turretRotation = getRobotRelitiveRotation();
        inputs.absTurretRotation = getRingRotation();

        inputs.canMakeItToTarget = canMakeItToTarget;
        inputs.limitSwitch = !limitSwitch.get();
    }

}
