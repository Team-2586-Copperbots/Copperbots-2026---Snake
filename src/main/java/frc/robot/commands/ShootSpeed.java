package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

public class ShootSpeed extends Command {
    private Shooter Shooter;
    private Double Speed;
    // true of false to use the setShooterSpeedAjust() command
    private boolean TFAjust;

    public ShootSpeed(Shooter shooterSubsystem, double speed, boolean ajust) {
        this.Shooter = shooterSubsystem;
        this.Speed = speed;
        this.TFAjust = ajust;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (TFAjust == true) {
            Shooter.setShooterSpeedAjust(Speed);
        } else if (TFAjust == false) {
            Shooter.setShooterSpeedSet(Speed);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Shooter.setShooterSpeedSet(Constants.SHOOTER_CONSTANTS.SHOOTER_IDLE_SPEED);
    }

}
