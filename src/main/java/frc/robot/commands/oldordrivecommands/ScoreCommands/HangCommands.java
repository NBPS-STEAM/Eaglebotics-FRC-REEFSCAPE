package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HangSubsystem;

public final class HangCommands {

    //Constructor to allow for each Command to reference the subsystem without a parameter
    private HangSubsystem hangSubsystem;

    public HangCommands(HangSubsystem hangSubsystem) {
        this.hangSubsystem = hangSubsystem;
    }



    //Positions the hang for grabbing onto the thing I forgot the name of
    public class Prepare extends Command {

        public Prepare(){
            addRequirements(hangSubsystem);
        }

        @Override 
        public void initialize() {
            // Nothing yet! Once the grippers are on, this will "activate" those, whatever that will do.
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }



    //Activates the hang routine
    public static class Activate extends Command {

        private final double targetPower;
        private double position;
        private final HangSubsystem hangSubsystem;
        private boolean toggleState = false;

        public Activate(HangSubsystem hangSubsystem, double power, double position){
            this.hangSubsystem = hangSubsystem;
            this.targetPower = power;
            this.position = position;
            addRequirements(hangSubsystem);
        }

        @Override 
        public void initialize() {
            if (!toggleState) {
                // First press: Move servos to position
                hangSubsystem.setServoPositions(position);
                toggleState = !toggleState;
            } else {
                // Second press: set the power
                hangSubsystem.setTwistPower(targetPower);
            }      
        }

        @Override 
        public boolean isFinished() { 
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            hangSubsystem.setTwistPower(0);
        }
    }



    //Resets the hang
    // public class Reset extends Command {

    //     public Reset(){
    //         addRequirements(hangSubsystem);
    //     }

    //     @Override 
    //     public void initialize() {
    //         hangSubsystem.setTwistPosition(0.0);
    //     }

    //     @Override 
    //     public boolean isFinished() { 
    //         return hangSubsystem.isAtTwistPosition();
    //     }
    // }
}
