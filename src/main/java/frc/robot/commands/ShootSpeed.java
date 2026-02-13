package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeed extends Command {
    private ShooterSubsystem ShooterSubsystem;
    private Double Speed;
    // true of false to use the setShooterSpeedAjust() command
    private boolean TFAjust;

    public ShootSpeed(ShooterSubsystem shooterSubsystem, double speed, boolean ajust) {
        this.ShooterSubsystem = shooterSubsystem;
        this.Speed = speed;
        this.TFAjust = ajust;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (TFAjust == true) {
            ShooterSubsystem.setShooterSpeedAjust(Speed);
        } else if (TFAjust == false) {
            ShooterSubsystem.setShooterSpeedSet(Speed);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // if (Math.abs(Shooter.getMotor1Speed() - Speed) < 0.5) {
        // return true;
        // }
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

}
