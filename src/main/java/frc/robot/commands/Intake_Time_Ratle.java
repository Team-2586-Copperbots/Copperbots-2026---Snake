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
    private boolean goingOut = false;
    private int timer = 0;

    public Intake_Time_Ratle(Intake Intake) {
        this.Intake = Intake;

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

        Logger.recordOutput("time betwen", System.currentTimeMillis() - lastJugleTime);
        // if (Math.abs(lastJugleTime - System.currentTimeMillis()) >
        // INTAKE_CONSTANTS.timeBetwenRattaling) {
        // Intake.setIntakePositionTarget(IntakePosition.OUT);
        // if (Intake.getIsAtTarget()) {
        // lastJugleTime = System.currentTimeMillis();
        // }
        // } else {
        // Intake.setIntakePositionTarget(IntakePosition.JUGGLE);
        // }
        if (timer < INTAKE_CONSTANTS.timeBetwenRattaling) {
            if (Intake.getIsAtTarget()) {
                Intake.setIntakePositionTarget(IntakePosition.JUGGLE);
            }
            timer++;
        } else {
            timer = 0;
            Intake.setIntakePositionTarget(IntakePosition.OUT);
        }

        // if (goingOut == true) {
        // if (Intake.getIsAtTarget()) {
        // goingOut = false;
        // }
        // } else {
        // Logger.recordOutput("Intake/time betwen, current based",
        // System.currentTimeMillis() - lastJugleTime);
        // if (System.currentTimeMillis()
        // - lastJugleTime > INTAKE_CONSTANTS.timeBetwenRattaling) {
        // Intake.setIntakePositionTarget(IntakePosition.OUT);
        // goingOut = true;
        // } else {
        // Intake.setIntakePositionTarget(IntakePosition.JUGGLE);
        // }
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
