package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.GeneralUtils;

public class Shooter_AutoSpeed extends Command {
    // class to bring the shooter up to speed, but not angle based on where the drive is
    private Shooter Shooter;

    public Shooter_AutoSpeed(Shooter shooterSubsystem) {
        this.Shooter = shooterSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Shooter.setShooterSpeedSet(
                GeneralUtils.shooterSpeedFromTarget());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Shooter.setShooterSpeedSet(Constants.OPERATOR_CONSTANTS.IDLE_SHOOTER_SPEED);
    }

}
