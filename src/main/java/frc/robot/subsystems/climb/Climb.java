package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIds;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

import static frc.robot.Constants.CANIds.Canivore;

import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private ClimbIO io;
    private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    public void setClimbSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setClimbPosition(ClimbPosition position) {
        io.setPosition(position);
    }

    public static enum ClimbPosition {
        UP(10),
        DOWN(0);

        private final double position;

        private ClimbPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

}
