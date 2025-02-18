package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePositionSubsystem;

public class StowCommand extends SequentialCommandGroup {
    IntakePositionCommand intakePositionCommand;
    public StowCommand(IntakePositionSubsystem intakePositionSubsystem){
        intakePositionCommand=new IntakePositionCommand(intakePositionSubsystem);
        addCommands(
            intakePositionCommand.new SetPivotSetpoint(Constants.IntakePositionConstants.stowPivot),
            intakePositionCommand.new SetLiftSetpoint(Constants.IntakePositionConstants.stowLift)
        );
    }
    
}
