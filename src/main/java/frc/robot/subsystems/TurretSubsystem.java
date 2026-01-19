package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import java.lang.Math;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.places;
import frc.robot.generated.TunerConstants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turnMotor;
    private final TalonFX spinnerMotor;
    private final TalonFXConfiguration turnMotorConfig;
    private final TalonFXConfiguration spinnerMotorConfig;
    private final CANcoder CANcoder;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    public TurretSubsystem() {
        turnMotor = new TalonFX(CANIds.TURRET_TURN_MOTOR);
        spinnerMotor = new TalonFX(CANIds.TURRET_SPIN_MOTOR);
        CANcoder = new CANcoder(0);
        turnMotorConfig = new TalonFXConfiguration();
        spinnerMotorConfig = new TalonFXConfiguration();

        var motorOutputConfigs = turnMotorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = motorOutputConfigs.NeutralMode.Coast;

        var pidConfig = turnMotorConfig.Slot0;
        // TODO: tune pid
        pidConfig.kP = 0.0;
        pidConfig.kI = 0.0;
        pidConfig.kD = 0.0;

        turnMotor.getConfigurator().apply(turnMotorConfig);
        spinnerMotor.getConfigurator().apply(spinnerMotorConfig);

        TurretConstants.setCANcoderOffset(CANcoder.getAbsolutePosition().getValueAsDouble());
    }

    // this uses the CTRE motors builtin constants to set the angle of the turret
    // with in the limits of 0-320 degreas
    public Command setintakePosition(Double angle) {
        if (angle > 0 && angle < 320) {
            return runOnce(() -> turnMotor.setControl(positionVoltage.withPosition(angle / 360)));
        } else {
            return runOnce(() -> turnMotor.setControl(positionVoltage.withPosition(0 / 360)));
        }
    }

    public Command aimFoward() {
        return setintakePosition(0.0);
    }

    public Command aimAtHub(CommandSwerveDrivetrain drivetrain) {
        double angle = 0;
        double robotAngle = drivetrain.getState().Pose.getRotation().getDegrees();
        SmartDashboard.putNumber("Robot Angle", robotAngle);
        double angleToHub = robotAngle + getAngleToHub(drivetrain);

        return setintakePosition(angleToHub);
    }

    // this returns the angle fron the center of the robot to the center of the hubs
    // schoeing element by way of math and arcsin()
    public double getAngleToHub(CommandSwerveDrivetrain drivetrain) {
        double angle = 0;
        Pose2d drivetrainPose2d = drivetrain.getState().Pose;
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

        SmartDashboard.putNumber("Angle to Hub", angle);
        return angle;
    }

    // this gets the angle of the cancoder sence the subsystem was initiated
    public double getTurretAngle() {
        return CANcoder.getAbsolutePosition().getValueAsDouble() - TurretConstants.CANCODER_OFFSET;
    }

    public void setSpinnerSpeed(double speed) {
        spinnerMotor.set(speed);
    }

    public double getMovementBarSpeed() {
        return turnMotor.getPosition().getValueAsDouble();
    }
}
