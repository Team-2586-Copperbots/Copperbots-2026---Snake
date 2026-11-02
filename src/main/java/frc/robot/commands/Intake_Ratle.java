package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE_CONSTANTS;
import frc.robot.Constants.OPERATOR_CONSTANTS;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class Intake_Ratle extends Command {
    // Oscillates the intake between OUT and JUGGLE positions to agitate balls into the spindexer
    // Uses an oscillating timer state machine with position-reached early triggers so it never hangs
    private enum RattleState {
        OUT,
        JUGGLE
    }

    private final Intake intake;
    private final Timer stateTimer = new Timer();
    private RattleState currentState = RattleState.OUT;

    private static final double RATTLE_PERIOD_SECS = INTAKE_CONSTANTS.timeBetwenRattaling / 1000.0; // 0.700s default
    private static final double MIN_HOLD_AT_TARGET_SECS = 0.25;

    public Intake_Ratle(Intake intakeSubsystem, boolean withCurrent) {
        this.intake = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    public Intake_Ratle(Intake intakeSubsystem) {
        this(intakeSubsystem, false);
    }

    @Override
    public void initialize() {
        currentState = RattleState.OUT;
        stateTimer.restart();
        intake.setRollerSpeed(OPERATOR_CONSTANTS.ROLLER_SPEED);
        intake.setIntakePositionTarget(IntakePosition.OUT);
    }

    @Override
    public void execute() {
        intake.setRollerSpeed(OPERATOR_CONSTANTS.ROLLER_SPEED);

        switch (currentState) {
            case OUT:
                intake.setIntakePositionTarget(IntakePosition.OUT);
                if (stateTimer.hasElapsed(RATTLE_PERIOD_SECS)
                        || (intake.getIsAtTarget() && stateTimer.hasElapsed(MIN_HOLD_AT_TARGET_SECS))) {
                    currentState = RattleState.JUGGLE;
                    stateTimer.restart();
                }
                break;

            case JUGGLE:
                intake.setIntakePositionTarget(IntakePosition.JUGGLE);
                if (stateTimer.hasElapsed(RATTLE_PERIOD_SECS)
                        || (intake.getIsAtTarget() && stateTimer.hasElapsed(MIN_HOLD_AT_TARGET_SECS))) {
                    currentState = RattleState.OUT;
                    stateTimer.restart();
                }
                break;
        }

        Logger.recordOutput("IntakeRattle/State", currentState.toString());
        Logger.recordOutput("IntakeRattle/ElapsedSecs", stateTimer.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        stateTimer.stop();
        intake.setIntakePositionTarget(IntakePosition.OUT);
    }
}
