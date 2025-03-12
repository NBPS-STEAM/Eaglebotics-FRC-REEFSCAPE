package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public final class IntakePositionCommand {

    //Constructor to allow for each Command to reference the subsystem without a parameter
    private IntakePositionSubsystem intakePositionSubsystem;

    public IntakePositionCommand(IntakePositionSubsystem intakePositionSubsystem) {
        this.intakePositionSubsystem = intakePositionSubsystem;
    }
    


    //Sets the elevator and the intake arm to a specified position, which is then reached through PIDs
    public class SetIntakePositionSetpoints extends Command {

        private double liftSetpoint;
        private double pivotSetpoint;

        public SetIntakePositionSetpoints(double liftSetpoint, double pivotSetpoint) {
            this.liftSetpoint = liftSetpoint;
            this.pivotSetpoint = pivotSetpoint;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setIntakePositionSetpoints(liftSetpoint, pivotSetpoint);
            if(liftSetpoint!=Constants.IntakePositionConstants.stowLift)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.atTargetPos();
        }
    }



    //Individual setter for the elevator
    public class SetLiftSetpoint extends Command {

        private double liftSetpoint;

        public SetLiftSetpoint(double liftSetpoint) {
            this.liftSetpoint = liftSetpoint;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setLiftSetpoint(liftSetpoint);
            if(liftSetpoint!=Constants.IntakePositionConstants.stowLift)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.liftAtTargetPos();
        }
    }



    //Individual setter for the intake arm
    public class SetPivotSetpoint extends Command {

        private double pivotSetpoint;

        public SetPivotSetpoint(double pivotSetpoint) {
            this.pivotSetpoint = pivotSetpoint;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setPivotSetpoint(pivotSetpoint);
            if(pivotSetpoint!=Constants.IntakePositionConstants.stowPivot)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.pivotAtTargetPos();
        }
    }



    /** Run the lift at velocity for duration of command */
    public class SetLiftVelocity extends Command {

        private double velocity;

        public SetLiftVelocity(double velocity) {
            this.velocity = velocity;
            addRequirements(intakePositionSubsystem);
        }

        @Override
        public void initialize() {
            intakePositionSubsystem.setLiftVelocity(velocity);
        }

        @Override
        public void end(boolean interrupted) {
            intakePositionSubsystem.setLiftVelocity(0);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }



    /** Run the pivot at velocity for duration of command */
    public class SetPivotVelocity extends Command {

        private double velocity;

        public SetPivotVelocity(double velocity) {
            this.velocity = velocity;
            addRequirements(intakePositionSubsystem);
        }

        @Override
        public void initialize() {
            intakePositionSubsystem.setPivotVelocity(velocity);
        }

        @Override
        public void end(boolean interrupted) {
            intakePositionSubsystem.setPivotVelocity(0);
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

}