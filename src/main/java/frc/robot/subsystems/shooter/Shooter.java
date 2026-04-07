package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.SHOOTER_CONSTANTS;

import static frc.robot.Constants.CANIds.SHOOTER_MOTOR_1;

import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;
    private ShooterIO io;
    private double setPoint = 0;

    public static Shooter getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    instance = new Shooter(new ShooterIOReal());
                    break;
                case SIM:
                    instance = new Shooter(new ShooterIOSim());
                    break;

                default:
                    instance = new Shooter(new ShooterIO() {
                    });
                    break;
            }
        }
        return instance;
    }

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    // negative to decrese
    public void setShooterSpeedAjust(double amount) {
        this.setPoint += amount;
        io.setMotorSetpoint(setPoint);
    }

    // sets the absolute speed
    public void setShooterSpeedSet(double setPoint) {
        this.setPoint = setPoint;
        io.setMotorSetpoint(setPoint);
    }

    public double getMotor1Speed() {
        return io.getMotorInputs(CANIds.SHOOTER_MOTOR_1).velocity;
    }

    @AutoLogOutput(key = "Shooter/getAtTarget")
    public boolean isAtTarget() {
        return Math.abs(getMotor1Speed() - io.getMotorInputs(SHOOTER_MOTOR_1).setpoint) < SHOOTER_CONSTANTS.TOLERENCE;
    }

    @Override
    public void periodic() {
        io.updateInputs(null);
    }
}
