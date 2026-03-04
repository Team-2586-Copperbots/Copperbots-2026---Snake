package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private double setPoint = 0;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    // negative to decrese
    public void setShooterSpeedAjust(double amount) {
        setPoint += amount;
        io.setMotorSetpoint(setPoint);
    }

    // sets the absolute speed
    public void setShooterSpeedSet(double setPoint) {
        setPoint = setPoint;
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
