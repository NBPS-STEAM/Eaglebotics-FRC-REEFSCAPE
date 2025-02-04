package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PipeIntakeSubsystem;

public final class PipeIntakeCommands {

    private PipeIntakeSubsystem pipeIntakeSubsystem;

    public PipeIntakeCommands(PipeIntakeSubsystem pipeIntakeSubsystem) {
        this.pipeIntakeSubsystem = pipeIntakeSubsystem;
    }

    public class Intake extends Command {

        public Intake(){
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

        public Outtake(){
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

        public StopIntake(){
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
