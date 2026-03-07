package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.GeneralUtils;

public class AutoSpeed extends Command {
    private Shooter Shooter;
    private Drive Drivetrain;

    public AutoSpeed(Shooter shooterSubsystem, Drive drivetrain) {
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
                GeneralUtils.shooterSpeedFromDistance(
                        GeneralUtils.distanceFromPose(Constants.FIELD_CONSTANTS.CENTER_OF_HUB, Drivetrain)));
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
