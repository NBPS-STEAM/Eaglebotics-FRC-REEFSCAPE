package frc.robot.commands.oldordrivecommands.ScoreCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BallIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.utils.Gamepieces;

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
            LEDSubsystem.getInstance().setIntake();
        }

        @Override
        public void end(boolean interrupted){
            if(!interrupted)ballIntakeSubsystem.setTargetVelocity(0.1);
            else ballIntakeSubsystem.setTargetVelocity(0);
            if (ballIntakeSubsystem.getHasBall()) {
                Gamepieces.gamepieceInRobot=Gamepieces.ALGAE;
                Gamepieces.activeGamepiece=Gamepieces.ALGAE;
            }
            LEDSubsystem.getInstance().setPlacePos();
        }

        @Override 
        public boolean isFinished() { 
            return ballIntakeSubsystem.getHasBall();
        }
    }



    //Turns on the ball outtake
    public class Outtake extends Command {

        private double time;
        private final double power;

        public Outtake() {
            this(Constants.IntakeConstants.kBallOuttakeSpeed);
        }

        public Outtake(double power) {
            this.power = power;
            addRequirements(ballIntakeSubsystem);
        }

        @Override 
        public void initialize() {
            LEDSubsystem.getInstance().setOuttake();
            ballIntakeSubsystem.setTargetVelocity(power);
            time=Timer.getFPGATimestamp()+0.5;
        }

        @Override
        public void end(boolean interrupted){
            ballIntakeSubsystem.setTargetVelocity(0);
            LEDSubsystem.getInstance().setPlacePos();
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
