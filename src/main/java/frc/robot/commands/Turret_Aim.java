package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.GeneralUtils;

public class Turret_Aim extends Command {
    private Turret Turret;

    public Turret_Aim(Turret TurretSubsystem) {
        this.Turret = TurretSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(TurretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Turret.setTurretRotationTarget(GeneralUtils.getAngleToTarget());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Turret.setTurretRotationTarget(0);
    }

}
