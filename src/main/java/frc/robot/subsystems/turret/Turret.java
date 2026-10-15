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
    private TurretIO io;
    private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    public static Turret getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case REAL:
                    instance = new Turret(new TurretIOReal());
                    break;
                case SIM:
                    instance = new Turret(new TurretIOSim());
                    break;

                default:
                    instance = new Turret(new TurretIO() {
                    });
                    break;
            }
        }
        return instance;
    }

    private Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
    }

    public void setTurretRotationTarget(Rotation2d rotation) {
        io.setTurretSetpoint(rotation);
    }

    public void setTurretRotationTarget(double rotations) {
        io.setTurretSetpoint(rotations);
    }

    public void setTurretSpeed(double speed) {
        io.setTurretSpeed(speed);
    }

    public Rotation2d getRotation() {
        return inputs.robotRelativeRotation;
    }

    public boolean getLimitSwitch() {
        return inputs.limitSwitch;
    }

    public boolean canGetToTarget() {
        return inputs.canMakeItToTarget;
    }

    public boolean isAtTarget() {
        Rotation2d current = inputs.robotRelativeRotation;
        Rotation2d target = inputs.targetRotation;
        double errorRotations = Math.abs(current.minus(target).getRotations());
        errorRotations = Math.min(errorRotations, 1.0 - errorRotations);
        return (errorRotations < TURRET_CONSTANTS.TOLERANCE.getRotations()) && canGetToTarget();
    }

    public void setTurretToZero() {
        io.setTurretZero();
    }

}
