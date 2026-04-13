package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;;

public class Turret_ManualTurret extends Command {
    // command to set the target angle of the turret manulay, proably for
    // trouble-shooting
    private Turret Turret;
    private double angle;

    public Turret_ManualTurret(Turret TurretSubsystem, double angle) {
        this.Turret = TurretSubsystem;
        this.angle = angle;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(TurretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Turret.setTurretRotationTarget(angle);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // if (Math.abs(Turret.getRingRotation() - angle) < 0.05) {
        // return true;
        // }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Turret.setTurretRotationTarget(0);
    }

}
