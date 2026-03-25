package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.CANIds;
import frc.robot.subsystems.climb.Climb.ClimbPosition;

public class ClimbIOReal implements ClimbIO {
    private final TalonFX climbMotor1, climbMotor2;
    private final TalonFXConfiguration climbMotorConfig;
    private boolean isPositionVoltage = false;
    private ClimbPosition targetPosition = ClimbPosition.DOWN;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    // private final PositionTorqueCurrentFOC

    public ClimbIOReal() {
        climbMotor1 = new TalonFX(CANIds.CLIMB_MOTOR_1);
        climbMotor2 = new TalonFX(CANIds.CLIMB_MOTOR_2);
        climbMotorConfig = new TalonFXConfiguration();

        var motorOutputConfigs = climbMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var pidConfig = climbMotorConfig.Slot0;
        pidConfig.kP = 0.500;
        pidConfig.kI = 0.000;
        pidConfig.kD = 0.000;

        climbMotor1.getConfigurator().apply(climbMotorConfig);
        climbMotor2.getConfigurator().apply(climbMotorConfig);

        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.motorPosition = climbMotor1.getPosition().getValueAsDouble();
        inputs.isPositionVoltage = isPositionVoltage;

        inputs.targetPosition = targetPosition;
        inputs.targetSpeed = climbMotor1.get();
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
