package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BallIntakeSubsystem;

public final class BallIntakeCommands {

    public class Intake extends Command {

        private final BallIntakeSubsystem ballIntakeSubsystem;

        public Intake(BallIntakeSubsystem ballIntakeSubsystem){
            this.ballIntakeSubsystem = ballIntakeSubsystem;
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

        private final BallIntakeSubsystem ballIntakeSubsystem;

        public Outtake(BallIntakeSubsystem ballIntakeSubsystem){
            this.ballIntakeSubsystem = ballIntakeSubsystem;
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

        private final BallIntakeSubsystem ballIntakeSubsystem;

        public StopIntake(BallIntakeSubsystem ballIntakeSubsystem){
            this.ballIntakeSubsystem = ballIntakeSubsystem;
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
