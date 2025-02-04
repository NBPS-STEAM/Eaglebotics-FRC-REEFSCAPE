package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BallIntakeSubsystem;

public final class BallIntakeCommands {

    private BallIntakeSubsystem ballIntakeSubsystem;

    public BallIntakeCommands(BallIntakeSubsystem ballIntakeSubsystem) {
        this.ballIntakeSubsystem = ballIntakeSubsystem;
    }

    public class Intake extends Command {
        
        public Intake() {
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            ballIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kBallIntakeSpeed);
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }

    public class Outtake extends Command {
        public Outtake() {
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            ballIntakeSubsystem.setTargetVelocity(Constants.IntakeConstants.kBallOuttakeSpeed);
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }

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
