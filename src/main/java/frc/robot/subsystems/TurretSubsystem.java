package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.lang.Math;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.places;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turnMotor;
    private final TalonFX spinnerMotor;
    private final TalonFXConfiguration turnMotorConfig;
    private final TalonFXConfiguration spinnerMotorConfig;
    private final CANcoder CANcoder;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private double angleToHub;
    private final Field2d drivetrainField;

    public TurretSubsystem() {
        turnMotor = new TalonFX(CANIds.TURRET_TURN_MOTOR);
        spinnerMotor = new TalonFX(CANIds.TURRET_SPIN_MOTOR);
        CANcoder = new CANcoder(CANIds.TURRET_CANCODER_ID);
        angleToHub = 0.0;
        drivetrainField = new Field2d();

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

    public Command setTurnMotorSpeed(double speed) {
        return runEnd(() -> turnMotor.set(speed), () -> turnMotor.set(0));
    }

    public Command setTurnMotorPosition(double rotation) {
        return runOnce(() -> turnMotor.setPosition(rotation));
    }

    // this uses the CTRE motors builtin constants to set the angle of the turret
    // with in the limits of 0-320 degreas
    public Command setIntakePosition(Double angle) {
        return runOnce(() -> turnMotor.setControl(positionVoltage.withPosition(angle * 11.2)));
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

    public Command aimAtHub(CommandSwerveDrivetrain drivetrain) {

        return setIntakePosition(getAngleToHub(drivetrain.getState().Pose));
    }

    // this returns the angle fron the center of the robot to the center of the hubs
    // schoeing element by way of math and arcsin()
    public double getAngleToHub(Pose2d drivetrainPose2d) {
        double angle = 0;
        drivetrainField.setRobotPose(drivetrainPose2d);
        Pose2d hubPose2d = places.CENTER_OF_HUB;
        if (hubPose2d.getX() - drivetrainPose2d.getX() > 0) {
            Pose2d relitiveHubPose2d = new Pose2d((drivetrainPose2d.getX() - hubPose2d.getX()),
                    (drivetrainPose2d.getY() - hubPose2d.getY()), null);

            angle = (Math.asin(Math.abs(relitiveHubPose2d.getY()) / Math.abs(relitiveHubPose2d.getX())) / Math.PI)
                    * 180;
            if (relitiveHubPose2d.getY() > 0) {
                angle = -angle;
            }
        }

        periodic();
        // System.out.println(angle);
        angle = angle / 360;

        double robotAngle = drivetrainPose2d.getRotation().getRotations();
        if (drivetrainPose2d.getY() > places.CENTER_OF_HUB.getY()) {
            angleToHub = robotAngle + angle;
        } else if (drivetrainPose2d.getY() < places.CENTER_OF_HUB.getY()) {
            angleToHub = -robotAngle + angle;
        } else {
            angleToHub = -robotAngle + angle;
        }
        angleToHub = -robotAngle + angle;

        return angleToHub;
    }

    // this gets the angle of the cancoder sence the subsystem was initiated

    public void setSpinnerSpeed(double speed) {
        spinnerMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("turret Motor", turnMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("turret encoder", CANcoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putString("positionVoltage", positionVoltage.getPositionMeasure().toLongString());
        SmartDashboard.putNumber("angle to hub", angleToHub);
        SmartDashboard.putData("drivtrain field from turret subsystem", drivetrainField);
        // SmartDashboard.putNumber("angle to hub", getAngleToHub());

    }
}
