package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.SHOOTER_CONSTANTS;
import frc.robot.util.auto_logging_stuff.SimMotorAutoLogged;
import frc.robot.util.auto_logging_stuff.TalonFXInputsAutoLogged;

public class ShooterIOSim extends ShooterIO {

    private TalonFXConfiguration config;
    private TalonFXSimState simState;
    private DCMotorSim motorSim;
    private LinearSystem linarsystem;
    private double moi = 0.001096772 * 2;

    public ShooterIOSim() {
        super();

        config = new TalonFXConfiguration();

        
        config.CurrentLimits.StatorCurrentLimit = SHOOTER_CONSTANTS.CURRENT_LIMIT;
        var motorOutputConfigs = config.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var pidConfig = config.Slot0;
        pidConfig.kP = 0.100;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;
        pidConfig.kV = 0.1125;

        shooterMotor1.getConfigurator().apply(config);
        shooterMotor2.getConfigurator().apply(config);
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        simState = shooterMotor1.getSimState();
        linarsystem = LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(2), moi, 1);
        motorSim = new DCMotorSim(linarsystem,
                DCMotor.getFalcon500(2), 0.01, 0.25);
                linarsystem.
    }

    @Override
    public void periodic() {
        simState.setSupplyVoltage(Volts.of(12.0));
        motorSim.setInputVoltage(simState.getMotorVoltage());

        motorSim.update(0.02);

        simState.setRawRotorPosition(motorSim.getAngularPosition());
        simState.setRotorVelocity(motorSim.getAngularVelocity());

        motor1Inputs.log("Shooter/Motor 1", simState, motorSim, shooterMotor1);

    }

    @Override
    public TalonFXInputsAutoLogged getMotorInputs(int id) {
        return motor1Inputs.getSimInputs(simState, motorSim, shooterMotor1);
    }
}
