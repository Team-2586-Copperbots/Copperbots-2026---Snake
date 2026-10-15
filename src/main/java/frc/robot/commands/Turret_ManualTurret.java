package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

public class Turret_ManualTurret extends Command {
    private final Turret turret;
    private final Rotation2d angle;

    public Turret_ManualTurret(Turret turretSubsystem, Rotation2d angle) {
        this.turret = turretSubsystem;
        this.angle = angle;
        addRequirements(turretSubsystem);
    }

    public Turret_ManualTurret(Turret turretSubsystem, double angleRotations) {
        this(turretSubsystem, Rotation2d.fromRotations(angleRotations));
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turret.setTurretRotationTarget(angle);
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
        turret.setTurretRotationTarget(Rotation2d.kZero);
    }

}
