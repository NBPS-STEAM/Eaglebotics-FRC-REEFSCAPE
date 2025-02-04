package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HangSubsystem;

public final class HangCommands {

    private HangSubsystem hangSubsystem;

    public HangCommands(HangSubsystem hangSubsystem) {
        this.hangSubsystem = hangSubsystem;
    }

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

    public class Activate extends Command {

        public Activate(){
            addRequirements(hangSubsystem);
        }

        @Override 
        public void initialize() {
            hangSubsystem.setTwistPosition(Constants.HangConstants.kHangTwistPosition);
        }

        @Override 
        public boolean isFinished() { 
            return hangSubsystem.isAtTwistPosition();
        }
    }

    public class Reset extends Command {

        public Reset(){
            addRequirements(hangSubsystem);
        }

        @Override 
        public void initialize() {
            hangSubsystem.setTwistPosition(0.0);
        }

        @Override 
        public boolean isFinished() { 
            return hangSubsystem.isAtTwistPosition();
        }
    }
}
