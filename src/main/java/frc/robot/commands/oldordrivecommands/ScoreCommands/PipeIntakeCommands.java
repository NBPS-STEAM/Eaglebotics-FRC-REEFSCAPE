package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PipeIntakeSubsystem;

public final class PipeIntakeCommands {

    //Constructor to allow for each Command to reference the subsystem without a parameter
    private PipeIntakeSubsystem pipeIntakeSubsystem;

    public PipeIntakeCommands(PipeIntakeSubsystem pipeIntakeSubsystem) {
        this.pipeIntakeSubsystem = pipeIntakeSubsystem;
    }



    //Turns on the pipe intake
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



    //Turns on the pipe outtake
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



    //Stops the intake/outtake
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
