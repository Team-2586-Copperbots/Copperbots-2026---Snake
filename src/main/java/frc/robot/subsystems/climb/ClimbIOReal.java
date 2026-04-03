package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    // private final PositionTorqueCurrentFOC


    private final MotorIOTalon[] motorio = new MotorIOTalon[2];
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    private final MotorIOInputsAutoLogged[] motorInputs = {new MotorIOInputsAutoLogged()};

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

        motorio[0] = new MotorIOTalon(climbMotor1);
        motorio[1] = new MotorIOTalon(climbMotor2);
    }

    @Override
    public void updateAndLogInputs() {
        inputs.motorPosition = climbMotor1.getPosition().getValueAsDouble();
        inputs.limitSwitch = !limitSwitch.get();

        // motorio[0].updateInputs(motorInputs[0]);

        // inputs.targetSpeed = climbMotor1.get();

        Logger.processInputs("Climb", inputs);
        // Logger.processInputs("Climb/Motor1", motorInputs[0]);
    }

    @Override
    public ClimbIOInputsAutoLogged getInputs() {
        return inputs;
    }
    @Override
    public MotorIOInputsAutoLogged getMotorInputs(int i) {
        return motorInputs[i];
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
