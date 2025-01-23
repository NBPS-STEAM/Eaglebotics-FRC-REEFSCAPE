package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PipeIntakeSubsystem;

public final class PipeIntakeCommands {

    public class Intake extends Command {

        private final PipeIntakeSubsystem pipeIntakeSubsystem;

        public Intake(PipeIntakeSubsystem pipeIntakeSubsystem){
            this.pipeIntakeSubsystem = pipeIntakeSubsystem;
            addRequirements(pipeIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            pipeIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kPipeIntakeSpeed);
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }

    public class Outtake extends Command {

        private final PipeIntakeSubsystem pipeIntakeSubsystem;

        public Outtake(PipeIntakeSubsystem pipeIntakeSubsystem){
            this.pipeIntakeSubsystem = pipeIntakeSubsystem;
            addRequirements(pipeIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            pipeIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kPipeOuttakeSpeed);
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }

    public class StopIntake extends Command {

        private final PipeIntakeSubsystem pipeIntakeSubsystem;

        public StopIntake(PipeIntakeSubsystem pipeIntakeSubsystem){
            this.pipeIntakeSubsystem = pipeIntakeSubsystem;
            addRequirements(pipeIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            pipeIntakeSubsystem.setTargetVelocity(0);
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }
}
