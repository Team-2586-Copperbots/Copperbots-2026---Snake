package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSpeed extends Command {
    private ShooterSubsystem Shooter;
    private CommandSwerveDrivetrain Drivetrain;

    public AutoSpeed(ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.Shooter = shooterSubsystem;
        this.Drivetrain = drivetrain;
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
                Utils.shooterSpeedFromDistance(Utils.distanceFromPose(Constants.PLACES.CENTER_OF_HUB, Drivetrain)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Shooter.setShooterSpeedSet(Constants.SHOOTER_CONSTANTS.SHOOTER_IDLE_SPEED);
    }

}
