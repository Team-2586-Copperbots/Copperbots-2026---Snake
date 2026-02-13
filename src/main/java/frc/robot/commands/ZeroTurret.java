package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class ZeroTurret extends Command {
    private TurretSubsystem Turret;

    public ZeroTurret(TurretSubsystem TurretSubsystem) {
        this.Turret = TurretSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(TurretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Turret.setTurnMotorSpeed(-0.10);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (Turret.getLimitSwitch()) {
            Turret.setTurnMotorPosition(0);
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Turret.setTurnMotorSpeed(0);
        Turret.setTurretRotation(0.0);
    }

}
