package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PIDShoot extends Command {
    ShooterSubsystem m_ShooterSubsystem;
    double m_speed;

    public PIDShoot(ShooterSubsystem shooterSubsystem, double speed) {
        m_ShooterSubsystem = shooterSubsystem;
        m_speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_ShooterSubsystem.resetPID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_ShooterSubsystem.placeholder(m_speed);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_ShooterSubsystem.getTargetShooterSpeed() - m_ShooterSubsystem.getShooterMotorSpeed()) < 0.05) {
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

}
