package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.GeneralUtils;

public class AimAtHub extends Command {
    private Turret Turret;
    private Drive Drivetrain;

    public AimAtHub(Turret TurretSubsystem, Drive Drivetrain) {
        this.Turret = TurretSubsystem;
        this.Drivetrain = Drivetrain;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(TurretSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Turret.setTurretRotationTarget(GeneralUtils.getAngleToHubWithVelocity(Drivetrain));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Turret.setTurretRotationTarget(Rotation2d.kZero);
    }

}
