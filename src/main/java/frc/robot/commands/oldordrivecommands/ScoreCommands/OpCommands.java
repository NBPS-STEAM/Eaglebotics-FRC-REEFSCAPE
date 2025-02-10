package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.commands.oldordrivecommands.ScoreCommands.IntakePositionCommand.SetIntakePositionSetpoints;
import frc.robot.commands.oldordrivecommands.ScoreCommands.IntakePositionCommand.SetLiftSetpoint;
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
    public Command getBall1Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball1Lift, Constants.OpConstantsForBall.Ball1Pivot);
    }

    public Command getBall2Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball2Lift, Constants.OpConstantsForBall.Ball2Pivot);
    }

    public Command getBall3Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball3Lift, Constants.OpConstantsForBall.Ball3Pivot);
    }

    public Command getBall4Command() {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            Constants.OpConstantsForBall.Ball4Lift, Constants.OpConstantsForBall.Ball4Pivot);
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

}