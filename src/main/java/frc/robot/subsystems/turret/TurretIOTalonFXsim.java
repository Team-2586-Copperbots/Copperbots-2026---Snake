package frc.robot.subsystems.turret;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.util.driveUtils.PhoenixUtil.TalonFXMotorControllerSim;

public class TurretIOTalonFXsim extends TurretIOTalonFX {
    private MapleMotorSim
    private TalonFXSimState sim;
    private DCMotorSim motorSim;
    private double moi = 0.0512119394;

    public TurretIOTalonFXsim() {
        super();
        sim = turnMotor.getSimState();
        motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), moi, TURRET_CONSTANTS.MOTOR_TO_RING_RATIO),
                DCMotor.getFalcon500(1), null);
    }

    @Override
    public void updateInputs() {
        SimulatedBattery.addMotor(null);
        sim.setSupplyVoltage(12);
        motorSim.setInputVoltage(sim.getMotorVoltage());

        motorSim.update(0.02);
        
        sim.setRawRotorPosition(motorSim.getAngularPosition());
        sim.setRotorVelocity(motorSim.getAngularVelocity());
        sim.setRotorAcceleration(motorSim.getAngularAcceleration());
    }

}
