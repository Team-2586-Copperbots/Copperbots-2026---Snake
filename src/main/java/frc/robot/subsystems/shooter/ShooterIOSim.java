package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import static frc.robot.Constants.CANIds.Canivore;

public class ShooterIOSim implements ShooterIO {
    private double velocityVoltagePlaceholder;

    public ShooterIOSim() {
        velocityVoltagePlaceholder = 0;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorSetpoint = velocityVoltagePlaceholder;
    }

    @Override
    public void setMotorSetpoint(double velocity) {
        velocityVoltagePlaceholder = velocity;
    }

    @Override
    public void setPercentageSpeed(double speed) {
        velocityVoltagePlaceholder = speed * 100;
    }
}
