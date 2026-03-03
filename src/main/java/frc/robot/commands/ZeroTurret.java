package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;;

public class ZeroTurret extends Command {
    private Turret Turret;

    public ZeroTurret(Turret TurretSubsystem) {
        this.Turret = TurretSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(TurretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Turret.setTurretSpeed(-0.10);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (Turret.getLimitSwitch()) {
            Turret.setTurretZero();
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Turret.setTurretSpeed(0);
    }

}
