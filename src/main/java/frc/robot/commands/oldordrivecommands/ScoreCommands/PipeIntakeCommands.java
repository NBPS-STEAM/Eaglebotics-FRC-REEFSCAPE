package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PipeIntakeSubsystem;
import frc.utils.Gamepieces;

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
            LEDSubsystem.getInstance().setIntake();
        }

        @Override
        public void end(boolean interrupted){
            pipeIntakeSubsystem.setTargetVelocity(0);
            LEDSubsystem.getInstance().setPlacePos();
        }

        @Override 
        public boolean isFinished() { 
            if(pipeIntakeSubsystem.getHasPipe()){
                Gamepieces.gamepieceInRobot=Gamepieces.CORAL;
                Gamepieces.activeGamepiece=Gamepieces.CORAL;
            }
            return pipeIntakeSubsystem.getHasPipe();
        }
    }



    //Turns on the pipe outtake
    public class Outtake extends Command {
        private double time;
        public Outtake(){
            addRequirements(pipeIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            time=Timer.getFPGATimestamp()+0.5;
            pipeIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kPipeOuttakeSpeed);
            LEDSubsystem.getInstance().setOuttake();
        }

        @Override
        public void end(boolean interrupted){
            pipeIntakeSubsystem.setTargetVelocity(0);
            LEDSubsystem.getInstance().setPlacePos();
        }

        @Override 
        public boolean isFinished() { 
            return Timer.getFPGATimestamp()>=time;
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
