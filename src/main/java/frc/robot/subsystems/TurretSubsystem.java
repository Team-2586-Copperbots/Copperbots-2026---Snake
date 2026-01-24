package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.TURRET_CONSTANTS;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turnMotor;
    private final TalonFX spinnerMotor;
    private final TalonFXConfiguration turnMotorConfig;
    private final TalonFXConfiguration spinnerMotorConfig;
    private final CANcoder CANcoder;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private double angleToHub;

    public TurretSubsystem() {
        turnMotor = new TalonFX(CANIds.TURRET_TURN_MOTOR);
        spinnerMotor = new TalonFX(CANIds.TURRET_SPIN_MOTOR);
        CANcoder = new CANcoder(CANIds.TURRET_CANCODER_ID);
        angleToHub = 0.0;

        turnMotorConfig = new TalonFXConfiguration();
        spinnerMotorConfig = new TalonFXConfiguration();

        // turnMotorConfig.Feedback.FeedbackRemoteSensorID = CANcoder.getDeviceID();
        // turnMotorConfig.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.RemoteCANcoder;
        // turnMotorConfig.Feedback.RotorToSensorRatio = 11.2;

        var motorOutputConfigs = turnMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        var pidConfig = turnMotorConfig.Slot0;
        // TODO: tune pid
        pidConfig.kP = 1.50;
        pidConfig.kI = 0.00;
        pidConfig.kD = 0.00;
        // pidConfig.kA = 0.00;
        // pidConfig.kG = 0.00;
        pidConfig.kS = 4.00;
        // pidConfig.kV = 0.00;

        turnMotor.getConfigurator().apply(turnMotorConfig);
        spinnerMotor.getConfigurator().apply(spinnerMotorConfig);

    }

    public Command setTurnMotor(double speed) {
        return runEnd(() -> turnMotor.set(speed), () -> turnMotor.set(0));
    }

    // this uses the CTRE motors builtin constants to set the angle of the turret
    // with in the limits of 0-320 degreas
    public Command setIntakePosition(Double angle) {
        return runOnce(() -> turnMotor.setControl(positionVoltage.withPosition(angle * TURRET_CONSTANTS.TURRET_MOTOR_TO_RING_RATIO)));
        // if (angle >= 0 && angle < 320) {
        // return runOnce(() -> turnMotor.setControl(positionVoltage.withPosition(angle
        // / 360)));
        // } else {
        // return runOnce(() -> turnMotor.setControl(positionVoltage.withPosition(0 /
        // 360)));
        // }
    }

    public Command aimFoward() {
        return setIntakePosition(0.0);
    }

    public void aimAtHub(CommandSwerveDrivetrain drivetrain) {
        double robotAngle = drivetrain.getState().Pose.getRotation().getRotations();
        angleToHub = robotAngle + Utils.getAngleToHub(drivetrain);
        setIntakePosition(angleToHub);
    }

    

    // this gets the angle of the cancoder sence the subsystem was initiated
    public double getTurretAngle() {
        return CANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setSpinnerSpeed(double speed) {
        spinnerMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret Motor", turnMotor.getDifferentialAveragePosition().getValueAsDouble());
        SmartDashboard.putNumber("turret encoder", CANcoder.getPositionSinceBoot().getValueAsDouble());
        SmartDashboard.putString("positionVoltage", positionVoltage.getPositionMeasure().toLongString());
        SmartDashboard.putNumber("angle to hub", angleToHub);

    }
}
