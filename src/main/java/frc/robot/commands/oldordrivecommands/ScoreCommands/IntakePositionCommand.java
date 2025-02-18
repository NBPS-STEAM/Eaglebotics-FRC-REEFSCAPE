package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePositionSubsystem;

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
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.pivotAtTargetPos();
        }
    }

}