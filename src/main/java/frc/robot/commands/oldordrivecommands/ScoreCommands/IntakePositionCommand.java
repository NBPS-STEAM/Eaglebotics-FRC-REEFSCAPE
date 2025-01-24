package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePositionSubsystem;

public final class IntakePositionCommand {

    public class SetIntakePositionSetpoints extends Command {

        private final IntakePositionSubsystem intakePositionSubsystem;
        private double liftSetpoint;
        private double pivotSetpoint;

        public SetIntakePositionSetpoints(IntakePositionSubsystem intakePositionSubsystem, double liftSetpoint, double pivotSetpoint) {
            this.intakePositionSubsystem = intakePositionSubsystem;
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

        private final IntakePositionSubsystem intakePositionSubsystem;
        private double liftSetpoint;

        public SetLiftSetpoint(IntakePositionSubsystem intakePositionSubsystem, double liftSetpoint) {
            this.intakePositionSubsystem = intakePositionSubsystem;
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

        private final IntakePositionSubsystem intakePositionSubsystem;
        private double pivotSetpoint;

        public SetPivotSetpoint(IntakePositionSubsystem intakePositionSubsystem, double pivotSetpoint) {
            this.intakePositionSubsystem = intakePositionSubsystem;
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