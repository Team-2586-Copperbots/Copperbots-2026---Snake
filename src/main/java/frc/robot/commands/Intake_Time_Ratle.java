package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.Constants.INTAKE_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.subsystems.intake.Intake;

public class Intake_Time_Ratle extends Command {
    // unfinished class to rattle the intake in and out for auto
    // needs to be runn constantly for the math with the intake roller to work
    private Intake Intake;
    private double lastJugleTime;
    // private boolean goingOut = false;

    public Intake_Time_Ratle(Intake Intake) {
        this.Intake = Intake;
        // make a implementation with current based along side time based

        lastJugleTime = System.currentTimeMillis();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        Intake.setRollerSpeed(OPERATOR_CONSTANTS.ROLLER_SPEED);

        Logger.recordOutput("time betwen", Math.abs(lastJugleTime - System.currentTimeMillis()));
        if (Math.abs(lastJugleTime - System.currentTimeMillis()) > INTAKE_CONSTANTS.timeBetwenRattaling) {
            Intake.setIntakePositionTarget(IntakePosition.OUT);
            if (Intake.getIsAtTarget()) {
                lastJugleTime = System.currentTimeMillis();
            }
        } else {
            Intake.setIntakePositionTarget(IntakePosition.JUGGLE);
        }

        // if () {
            
        // }



    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Intake.setIntakePositionTarget(IntakePosition.OUT);
    }

}
