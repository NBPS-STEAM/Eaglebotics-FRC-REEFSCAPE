package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePositionSubsystem;

public final class IntakePositionCommand {

    private IntakePositionSubsystem intakePositionSubsystem;

    public IntakePositionCommand(IntakePositionSubsystem intakePositionSubsystem) {
        this.intakePositionSubsystem = intakePositionSubsystem;
    }
    
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
            return true;
        }
    }




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
            return true;
        }
    }




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
            return true;
        }
    }

}