package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class OpCommands {

    // STATIC

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
            () -> gamepad.getRightX());

        return driveFieldOrientedAnglularVelocity;


    }

    public static Command getSlowDriveCommand(SwerveSubsystem drivebase, CommandPS5Controller gamepad) {
        // same as above but half as fast
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
            () -> MathUtil.applyDeadband(gamepad.getLeftY(),  Constants.OIConstants.kDriveDeadband) * Constants.DriveConstants.speedFactor,
            () -> MathUtil.applyDeadband(gamepad.getLeftX(),  Constants.OIConstants.kDriveDeadband) * Constants.DriveConstants.speedFactor,
            () -> gamepad.getRightX() * Constants.DriveConstants.speedFactor);

        return driveFieldOrientedAnglularVelocity;

    
    }



    // INSTANCE


    //Constructor to allow for each Command to reference the subsystem without a parameter
    private final IntakePositionSubsystem intakePositionSubsystem;
    private final IntakePositionCommand intakePositionCommand;

    public OpCommands(IntakePositionSubsystem intakePositionSubsystem) {
        this.intakePositionSubsystem = intakePositionSubsystem;
        intakePositionCommand = new IntakePositionCommand(intakePositionSubsystem);
    }


    //Individual commands for each set position needed for the four levels of the ball and pipe

    /**
     * Move lift and pivot simultaneously to the stow position.
     */
    public ParallelCommandGroup getStowParallelCommand() {
        return new ParallelCommandGroup(intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.IntakePositionConstants.stowLift, Constants.IntakePositionConstants.stowPivot),
            new InstantCommand(()->LEDSubsystem.getInstance().setStow())
            );
    }

    /**
     * Move lift and pivot simultaneously to the barge shooting position.
     */
    public ParallelCommandGroup getBargeShootCommand() {
        return new ParallelCommandGroup(intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.IntakePositionConstants.bargeLift, Constants.IntakePositionConstants.bargePivot),
            new InstantCommand(()->LEDSubsystem.getInstance().setBarge())
            );
    }

    /**
     * Move lift and pivot simultaneously to the ball ground intake position.
     * @see #ballCommandGroup(int)
     */
    public Command getBall1Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball1Lift, Constants.OpConstantsForBall.Ball1Pivot);
    }

    /**
     * Move lift and pivot simultaneously to the ball processor scoring position.
     * @see #ballCommandGroup(int)
     */
    public Command getBall2Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball2Lift, Constants.OpConstantsForBall.Ball2Pivot);
    }

    /**
     * Move lift and pivot simultaneously to the ball level 2 intake position.
     * @see #ballCommandGroup(int)
     */
    public Command getBall3Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball3Lift, Constants.OpConstantsForBall.Ball3Pivot);
    }

    /**
     * Move lift and pivot simultaneously to the ball level 3 intake position.
     * @see #ballCommandGroup(int)
     */
    public Command getBall4Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball4Lift, Constants.OpConstantsForBall.Ball4Pivot);
    }

    /** Move lift and pivot simultaneously to the pipe intake position. */
    public Command getPipeIntakeCommand() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.PipeIntakeLift, Constants.OpConstantsForPipe.PipeIntakePivot);
    }

    /**
     * Move lift and pivot simultaneously to the pipe level 1 score position.
     * @see #pipeCommandGroup(int)
     */
    public Command getPipe1Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe1Lift, Constants.OpConstantsForPipe.Pipe1Pivot);
    }

    /**
     * Move lift and pivot simultaneously to the pipe level 2 score position.
     * @see #pipeCommandGroup(int)
     */
    public Command getPipe2Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe2Lift, Constants.OpConstantsForPipe.Pipe2Pivot);
    }

    /**
     * Move lift and pivot simultaneously to the pipe level 3 score position.
     * @see #pipeCommandGroup(int)
     */
    public Command getPipe3Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe3Lift, Constants.OpConstantsForPipe.Pipe3Pivot);
    }

    /**
     * Move lift and pivot simultaneously to the pipe level 4 score position.
     * @see #pipeCommandGroup(int)
     */
    public Command getPipe4Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForPipe.Pipe4Lift, Constants.OpConstantsForPipe.Pipe4Pivot);
    }


    


    //basic command groups

    /**
     * Move first the lift, then the pivot to a set position for pipes.
     * @param level The level to score at (within 1-4, otherwise will go to stow position)
     */
    public SequentialCommandGroup pipeCommandGroup(int level){
        SequentialCommandGroup command;
        switch (level) {
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

    /**
     * Move first the lift, then the pivot to a set position for balls.
     * <p>Levels:</p>
     * <p>1 - Ground intake</p>
     * <p>2 - Proccessor</p>
     * <p>3 - Level 2 intake</p>
     * <p>4 - Level 3 intake</p>
     * <p>Anything else - Stow position</p>
     */
    public SequentialCommandGroup ballCommandGroup(int level){
        SequentialCommandGroup command;
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