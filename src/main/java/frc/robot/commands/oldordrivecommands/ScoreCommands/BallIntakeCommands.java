package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.utils.GampiecesFsm;
import frc.utils.GampiecesFsm.Gamepieces;

public final class BallIntakeCommands {

    //Constructor to allow for each Command to reference the subsystem without a parameter
    private BallIntakeSubsystem ballIntakeSubsystem;

    public BallIntakeCommands(BallIntakeSubsystem ballIntakeSubsystem) {
        this.ballIntakeSubsystem = ballIntakeSubsystem;
    }



    //Turns on the ball intake
    public class Intake extends Command {
        
        public Intake() {
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            ballIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kBallIntakeSpeed);
        }

        @Override
        public void end(boolean interrupted){
            ballIntakeSubsystem.setTargetVelocity(0);
        }

        @Override 
        public boolean isFinished() { 
            if(ballIntakeSubsystem.getHasBall()){
                GampiecesFsm.gamepieceInRobot=Gamepieces.ALGAE;
                GampiecesFsm.activeGamepiece=Gamepieces.ALGAE;
            }
            return ballIntakeSubsystem.getHasBall();
        }
    }



    //Turns on the ball outtake
    public class Outtake extends Command {
        private double time;
        public Outtake() {
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            ballIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kBallOuttakeSpeed);
            time=Timer.getFPGATimestamp()+0.5;
        }

        @Override
        public void end(boolean interrupted){
            ballIntakeSubsystem.setTargetVelocity(0);
        }

        @Override 
        public boolean isFinished() { 
        
            return Timer.getFPGATimestamp()>=time;
        }
    }



    //Stops the intake/outtake
    public class StopIntake extends Command {
        public StopIntake() {
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            ballIntakeSubsystem.setTargetVelocity(0);
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }
}
