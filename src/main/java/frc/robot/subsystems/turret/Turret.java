package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TURRET_CONSTANTS;

//
//
// this subsystem works in rotations!!!
//
//

public class Turret extends SubsystemBase {
    private static Turret instance = null;
    private TurretIOTalonFX io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public static Turret getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    instance = new Turret(new TurretIOTalonFX());
                    break;
                case SIM:
                    instance = new Turret(new TurretIOTalonFXsim());
                    break;

                default:
                    instance = new Turret(new TurretIOTalonFX() {
                    });
                    break;
            }
        }
        return instance;
    }

    private Turret(TurretIOTalonFX io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs();
        Logger.processInputs("Turret", inputs);
    }

    public void setTurretRotationTarget(double roations) {
        io.setTurretSetpoint(roations);
    }

    public void setTurretSpeed(double speed) {
        io.setTurretSpeed(speed);
    }

    public Rotation2d getRotation() {
        return new Rotation2d(Rotations.of(io.getRobotRelitiveRotation()));
    }

    public boolean getLimitSwitch() {
        return inputs.limitSwitch;
    }

    public boolean canGetToTarget() {
        return inputs.canMakeItToTarget;
    }

    public boolean isAtTarget() {
        return (Math.abs(io.getMotorInputs().position - io.getMotorInputs().setpoint) < TURRET_CONSTANTS.TOLERENCE)
                && canGetToTarget();
    }

    public void setTurretToZero() {
        io.setTurretZero();
    }

}
