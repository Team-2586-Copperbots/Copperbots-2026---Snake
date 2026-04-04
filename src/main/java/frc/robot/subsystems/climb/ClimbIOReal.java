package frc.robot.subsystems.climb;

import java.io.File;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.subsystems.climb.Climb.ClimbPosition;
import frc.robot.util.auto_loggint_stuff.MotorIOInputsAutoLogged;
import frc.robot.util.auto_loggint_stuff.MotorIOTalon;
import frc.robot.util.auto_loggint_stuff.MotorIO.MotorIOInputs;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX climbMotor1, climbMotor2;
    private final DigitalInput limitSwitch;
    private final TalonFXConfiguration climbMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    // private final SparkMax motot = new SparkMax(0, MotorType.kBrushless)
    // private final PositionTorqueCurrentFOC

    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final MotorIOTalon climbMotor1IO, climbMotor2IO;
    private final MotorIOInputsAutoLogged climbMotor1Inputs = new MotorIOInputsAutoLogged();
    private final MotorIOInputsAutoLogged climbMotor2Inputs = new MotorIOInputsAutoLogged();

    public ClimbIOReal() {
        climbMotor1 = new TalonFX(CANIds.CLIMB_MOTOR_1);
        climbMotor2 = new TalonFX(CANIds.CLIMB_MOTOR_2);
        limitSwitch = new DigitalInput(DIO_IDS.CLIMB_LIMIT_SWITCH);
        climbMotorConfig = new TalonFXConfiguration();

        var motorOutputConfigs = climbMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var pidConfig = climbMotorConfig.Slot0;
        // TODO: tune pid
        pidConfig.kP = 2.000;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;
        // pidConfig.kG = 0.005;
        // pidConfig.GravityType = GravityTypeValue.Elevator_Static;

        climbMotor1.getConfigurator().apply(climbMotorConfig);
        climbMotor2.getConfigurator().apply(climbMotorConfig);

        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), MotorAlignmentValue.Aligned));

        climbMotor1IO = new MotorIOTalon(climbMotor1);
        climbMotor2IO = new MotorIOTalon(climbMotor2);
    }

    @Override
    public void updateAndLogInputs() {
        inputs.limitSwitch = !limitSwitch.get();

        Logger.processInputs("Climb", inputs);

        climbMotor1IO.updateInputs(climbMotor1Inputs);
        climbMotor2IO.updateInputs(climbMotor2Inputs);
        Logger.processInputs("Climb/Climb Motor 1", climbMotor1Inputs);
        Logger.processInputs("Climb/Climb Motor 2", climbMotor2Inputs);

        // for (int i = 0; i < motorio.length; i++) {
        // motorio[i].updateInputs(motorInputs[i]);
        // Logger.processInputs("Climb/motor"+i, motorInputs[i]);
        // }
    }

    @Override
    public ClimbIOInputsAutoLogged getInputs() {
        return inputs;
    }

    @Override
    public MotorIOInputsAutoLogged getMotorInputs(int id) {
        switch (id) {
            case CANIds.CLIMB_MOTOR_1:
                return climbMotor1Inputs;
            case CANIds.CLIMB_MOTOR_2:
                return climbMotor2Inputs;
            default:
                return null;
        }
    }

    @Override
    public void setPosition(double position) {
        climbMotor1.setPosition(position);
        climbMotor2.setPosition(position);
    }

    @Override
    public void setSpeed(double speed) {
        climbMotor1.set(speed);
    }

    @Override
    public void setTargetPosition(ClimbPosition position) {
        climbMotor1.setControl(positionVoltage.withPosition(position.value));
    }
}
