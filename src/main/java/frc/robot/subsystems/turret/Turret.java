package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public Rotation2d getRobotRelitiveRotation2D() {
        return new Rotation2d(Angle.ofBaseUnits(getRobotRelitiveRotation(), Rotation));
    }

    public void setTurretToZero() {
        io.setTurretZero();
    }

}
