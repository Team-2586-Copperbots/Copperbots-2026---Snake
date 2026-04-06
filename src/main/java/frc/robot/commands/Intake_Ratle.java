package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.Constants.INTAKE_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.subsystems.intake.Intake;

public class Intake_Ratle extends Command {
    private Intake Intake;
    private double lastActionTime;

    public Intake_Ratle(Intake Intake) {
        this.Intake = Intake;
        lastActionTime = System.currentTimeMillis();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Intake.setRollerSpeed(OPERATOR_CONSTANTS.ROLLER_SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Logger.recordOutput("time betwen", Math.abs(lastActionTime - System.currentTimeMillis()));
        if (Math.abs(lastActionTime - System.currentTimeMillis()) > INTAKE_CONSTANTS.timeBetwenRattaling) {
            Intake.setIntakePositionTarget(IntakePosition.OUT);
            if (Intake.getIsAtTArget()) {
                lastActionTime = System.currentTimeMillis();
            }
        } else {
            Intake.setIntakePositionTarget(IntakePosition.JUGLE);
        }

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
