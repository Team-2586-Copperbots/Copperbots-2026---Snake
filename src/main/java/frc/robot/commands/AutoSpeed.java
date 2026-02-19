package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Utils;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSpeed extends Command {
    private ShooterSubsystem ShooterSubsystem;
    private CommandSwerveDrivetrain Drivetrain;
    // true of false to use the setShooterSpeedAjust() command
    private boolean TFAjust;

    public AutoSpeed(ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.ShooterSubsystem = shooterSubsystem;
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
        ShooterSubsystem.setShooterSpeedSet(
                Utils.shooterSpeedFromDistance(Utils.distanceFromPose(Constants.PLACES.CENTER_OF_HUB, Drivetrain)));
    }

    @Override
    public boolean isFinished() {
        // if (Math.abs(Shooter.getMotor1Speed() - Speed) < 0.5) {
        // return true;
        // }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

}
