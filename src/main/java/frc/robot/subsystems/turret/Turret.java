package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

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

    public void setTurretToZero() {
        io.setTurretSpeed(0);
        io.setTurretZero();
        io.setTurretSetpoint(0);
    }

}
