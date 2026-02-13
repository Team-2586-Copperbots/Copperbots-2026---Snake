package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class AimAtHub extends Command {
    private TurretSubsystem Turret;
    private CommandSwerveDrivetrain Drivetrain;

    public AimAtHub(TurretSubsystem TurretSubsystem, CommandSwerveDrivetrain Drivetrain) {
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
        // if (Math.abs(Turret.getRingRotation() - angle) < 0.05) {
        // return true;
        // }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

}
