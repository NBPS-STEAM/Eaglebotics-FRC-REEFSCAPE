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




    static IntakePositionCommand intakePositionCommand = new IntakePositionCommand();


    public static Command getBall1Command(IntakePositionSubsystem intakePositionSubsystem, CommandPS5Controller gamepad) {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            intakePositionSubsystem, Constants.OpConstantsForBall.Ball1Lift, Constants.OpConstantsForBall.Ball1Pivot);
    }

    public static Command getBall2Command(IntakePositionSubsystem intakePositionSubsystem, CommandPS5Controller gamepad) {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            intakePositionSubsystem, Constants.OpConstantsForBall.Ball2Lift, Constants.OpConstantsForBall.Ball2Pivot);
    }

    public static Command getBall3Command(IntakePositionSubsystem intakePositionSubsystem, CommandPS5Controller gamepad) {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            intakePositionSubsystem, Constants.OpConstantsForBall.Ball3Lift, Constants.OpConstantsForBall.Ball3Pivot);
    }

    public static Command getBall4Command(IntakePositionSubsystem intakePositionSubsystem, CommandPS5Controller gamepad) {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            intakePositionSubsystem, Constants.OpConstantsForBall.Ball4Lift, Constants.OpConstantsForBall.Ball4Pivot);
    }


    public static Command getPipe1Command(IntakePositionSubsystem intakePositionSubsystem, CommandPS5Controller gamepad) {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            intakePositionSubsystem, Constants.OpConstantsForPipe.Pipe1Lift, Constants.OpConstantsForPipe.Pipe1Pivot);
    }

    public static Command getPipe2Command(IntakePositionSubsystem intakePositionSubsystem, CommandPS5Controller gamepad) {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            intakePositionSubsystem, Constants.OpConstantsForPipe.Pipe2Lift, Constants.OpConstantsForPipe.Pipe2Pivot);
    }

    public static Command getPipe3Command(IntakePositionSubsystem intakePositionSubsystem, CommandPS5Controller gamepad) {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            intakePositionSubsystem, Constants.OpConstantsForPipe.Pipe3Lift, Constants.OpConstantsForPipe.Pipe3Pivot);
    }

    public static Command getPipe4Command(IntakePositionSubsystem intakePositionSubsystem, CommandPS5Controller gamepad) {
        return intakePositionCommand.new SetIntakePositionSetpoints(
            intakePositionSubsystem, Constants.OpConstantsForPipe.Pipe4Lift, Constants.OpConstantsForPipe.Pipe4Pivot);
    }

}
