package frc.robot.subsystems.turret;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DIO_IDS;
import frc.robot.Constants.TURRET_CONSTANTS;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

//
//
// this subsystem works in rotations!!!
//
//

public class Turret extends SubsystemBase {
    private TurretIO io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public void setTurretRotationTarget(double roations) {
        io.setTurretSetpoint(roations);
    }

    public void setTurretSpeed(double speed) {
        io.setTurretSpeed(speed);
    }

    public boolean getLimitSwitch() {
        return inputs.limitSwitch;
    }

    public double getRobotRelitiveRotation() {
        return inputs.rotationRelitiveToRobotZero;
    }

    public void setTurretZero() {
        io.setTurretSpeed(0);
        io.setTurretZero();
    }

}
