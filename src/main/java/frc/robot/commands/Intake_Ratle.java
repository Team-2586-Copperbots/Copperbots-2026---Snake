// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.intake.Intake.IntakePosition;
// import frc.robot.Constants;
// import frc.robot.subsystems.intake.Intake;

// public class Intake_Ratle extends Command {
//     private Intake Intake;

//     public Intake_Ratle(Intake Intake) {
//         this.Intake = Intake;
//         // Use addRequirements() here to declare subsystem dependencies.
//         addRequirements(Intake);
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         Intake.setRollerSpeed(Constants.OPERATOR_CONSTANTS.ROLLER_SPEED);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         if (Intake.isAtTarget() && Intake.getWristTarget() == IntakePosition.HALFWAY) {
//             Intake.setIntakePositionTarget(IntakePosition.OUT);
//         } else if (Intake.isAtTarget() && Intake.getWristTarget() == IntakePosition.OUT) {
//             Intake.setIntakePositionTarget(IntakePosition.HALFWAY);
//         }

//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         Intake.setIntakePositionTarget(IntakePosition.OUT);
//     }

// }
