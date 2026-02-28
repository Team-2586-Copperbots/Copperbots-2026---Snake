package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX movementMotor;
    private final TalonFX spinnerMotor;
    private final CANcoder cancoder;
    private final TalonFXConfiguration movementMotorConfig;
    private final TalonFXConfiguration spinnerMotorConfig;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);

    public 
}
