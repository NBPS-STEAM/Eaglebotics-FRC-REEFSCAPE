package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class OpCommands {

    public static Command getDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad) {
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        /* Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(driverGamepad.getLeftY(), Constants.OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(driverGamepad.getLeftX(), Constants.OIConstants.kDriveDeadband),
            () -> driverGamepad.getRightX(),
            () -> driverGamepad.getRightY()); */

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(gamepad.getLeftY(),  Constants.OIConstants.kDriveDeadband),
            () -> MathUtil.applyDeadband(gamepad.getLeftX(),  Constants.OIConstants.kDriveDeadband),
            () -> gamepad.getRightX() * 0.5);

        return driveFieldOrientedAnglularVelocity;


    }

    public static Command getSlowDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad) {
        // same as above but half as fast
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(gamepad.getLeftY(),  Constants.OIConstants.kDriveDeadband) * Constants.DriveConstants.speedFactor,
            () -> MathUtil.applyDeadband(gamepad.getLeftX(),  Constants.OIConstants.kDriveDeadband) * Constants.DriveConstants.speedFactor,
            () -> gamepad.getRightX() * (0.5 * Constants.DriveConstants.speedFactor));

        return driveFieldOrientedAnglularVelocity;

    
    }





    //Constructor to allow for each Command to reference the subsystem without a parameter
    private IntakePositionSubsystem intakePositionSubsystem;

    public OpCommands(IntakePositionSubsystem intakePositionSubsystem) {
        this.intakePositionSubsystem = intakePositionSubsystem;
    }

    IntakePositionCommand intakePositionCommand = new IntakePositionCommand(intakePositionSubsystem);


    //Individual commands for each set position needed for the four levels of the ball and pipe
    public Command getBall1Command() {//ground intake
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball1Lift, Constants.OpConstantsForBall.Ball1Pivot);
    }

    public Command getBall2Command() {//prossesor score
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball2Lift, Constants.OpConstantsForBall.Ball2Pivot);
    }

    public Command getBall3Command() {//level 2 intake
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball3Lift, Constants.OpConstantsForBall.Ball3Pivot);
    }

    public Command getBall4Command() {//level 3 intake
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball4Lift, Constants.OpConstantsForBall.Ball4Pivot);
    }

    public Command getPipeIntakeCommand() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.PipeIntakeLift, Constants.OpConstantsForPipe.PipeIntakePivot);
    }

    public Command getPipe1Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe1Lift, Constants.OpConstantsForPipe.Pipe1Pivot);
    }

    public Command getPipe2Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe2Lift, Constants.OpConstantsForPipe.Pipe2Pivot);
    }

    public Command getPipe3Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe3Lift, Constants.OpConstantsForPipe.Pipe3Pivot);
    }

    public Command getPipe4Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe4Lift, Constants.OpConstantsForPipe.Pipe4Pivot);
    }

    //baisic command groups
    public SequentialCommandGroup coralCommandGroup(int level){
        SequentialCommandGroup command;
        switch (level) {//number corresponds to scoring level
            case 1:
                command= new SequentialCommandGroup(
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForPipe.Pipe1Lift),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe1Pivot)
                );
                break;
            case 2:
                command= new SequentialCommandGroup(
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForPipe.Pipe2Lift),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe2Pivot)
                );
                
                break;
            case 3:
                 command= new SequentialCommandGroup(
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForPipe.Pipe3Lift),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe3Pivot)
                );
                
                break;
            case 4:
                command= new SequentialCommandGroup(
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForPipe.Pipe4Lift),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForPipe.Pipe4Pivot)
                );
                
                break;
            default:
                command=new StowCommand(intakePositionSubsystem);
                break;
            
        }
        return command;
    }

    public SequentialCommandGroup ballCommandGroup(int level){//1 for ground intake , 2 for proccesser
        SequentialCommandGroup command;//3 for level 2 intake, 3 for level 3 intake
        switch (level) {
            case 1:
                command= new SequentialCommandGroup(
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball1Lift),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball1Pivot)
                );
                break;
            case 2:
                command= new SequentialCommandGroup(
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball2Lift),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball2Pivot)
                );
                
                break;
            case 3:
                 command= new SequentialCommandGroup(
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball3Lift),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball3Pivot)
                );
                
                break;
            case 4:
                command= new SequentialCommandGroup(
                    intakePositionCommand. new SetLiftSetpoint(Constants.OpConstantsForBall.Ball4Lift),
                    intakePositionCommand. new SetPivotSetpoint(Constants.OpConstantsForBall.Ball4Pivot)
                );
                
                break;
            default:
                command= new StowCommand(intakePositionSubsystem);
                break;
            
        }
        return command;
    }

}