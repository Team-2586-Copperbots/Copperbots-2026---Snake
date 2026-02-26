package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.drive.Drive;

public class AimAtHub extends Command {
    private TurretSubsystem Turret;
    private Drive Drivetrain;

    public AimAtHub(TurretSubsystem TurretSubsystem, Drive Drivetrain) {
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
        Turret.setTurretRotation(Utils.getAngleToHub(Drivetrain));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Turret.setTurretRotation(0);
    }

}
