package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
    private ShooterSubsystem Shooter;
    private Double Speed;

    public Shoot(ShooterSubsystem ShooterSubsystem, double speed) {
        this.Shooter = ShooterSubsystem;
        this.Speed = speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ShooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Shooter.setShooterSpeed(Speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        if (Math.abs(Shooter.getCurrentMotorSpeed() - Speed) < 0.5) {
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

}
