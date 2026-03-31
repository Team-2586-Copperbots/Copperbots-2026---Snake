package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
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
        return inputs.currentMotorSpeed;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }
}
