package frc.robot.subsystems.intake;

import static frc.robot.Constants.CANIds.Canivore;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    // Constants.IntakePosition
    public void setIntakePosition(IntakePosition position) {
        inputs.isClosedLoop = true;
        inputs.wristSetpoint = position;
    }

    // positive is out
    public void setMovementBarSpeed(double speed) {
        inputs.isClosedLoop = false;
        inputs.percentageWristSpeed = speed;
    }

    public void setRollerSpeed(double speed) {
        inputs.rollerSetpoint = speed;
    }

    public double getMovementBarPosition() {
        return inputs.currentWristPosition;
    }

    public boolean getIsDown() {
        double threshold = 0.05;
        if (((inputs.currentWristPosition - IntakePosition.IN.value) < threshold)
                || ((inputs.currentWristPosition - IntakePosition.OUT.value) < threshold)) {
            return true;
        } else {
            return false;
        }

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake position in rotations", getMovementBarPosition());
    }

    public static enum IntakePosition {
        IN(0),
        OUT(1),
        HALFWAY(0.5);

        public final double value;

        private IntakePosition(double value) {
            this.value = value;
        }
    }
}
