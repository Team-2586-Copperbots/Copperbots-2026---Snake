package frc.robot.subsystems.climb;

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

public class ClimbIOReal implements ClimbIO {
    private final TalonFX climbMotor1, climbMotor2;
    private final DigitalInput limitSwitch;
    private final TalonFXConfiguration climbMotorConfig;
    private boolean isPositionVoltage = false;
    private ClimbPosition targetPosition = ClimbPosition.DOWN;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    // private final PositionTorqueCurrentFOC

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
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorPosition = climbMotor1.getPosition().getValueAsDouble();
        inputs.isPositionVoltage = isPositionVoltage;
        inputs.limitSwitch = !limitSwitch.get();

        inputs.targetPosition = targetPosition;
        inputs.targetSpeed = climbMotor1.get();
    }

    @Override
    public void setPosition(double position) {
        climbMotor1.setPosition(position);
        climbMotor2.setPosition(position);
    }

    @Override
    public void setSpeed(double speed) {
        isPositionVoltage = false;
        climbMotor1.set(speed);
    }

    @Override
    public void setTargetPosition(ClimbPosition position) {
        targetPosition = position;
        isPositionVoltage = true;
        climbMotor1.setControl(positionVoltage.withPosition(position.value));
    }
}
