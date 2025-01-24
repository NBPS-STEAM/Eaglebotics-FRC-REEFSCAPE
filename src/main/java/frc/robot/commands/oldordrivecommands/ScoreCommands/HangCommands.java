package frc.robot.commands.oldordrivecommands.ScoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HangSubsystem;

public final class HangCommands {

    public class Prepare extends Command {

        private final HangSubsystem hangSubsystem;

        public Prepare(HangSubsystem hangSubsystem){
            this.hangSubsystem = hangSubsystem;
            addRequirements(hangSubsystem);
        }

        @Override 
        public void initialize() {
            // Nothing yet!
            // Once the grippers are on, this will "activate" those, whatever that will do.
        }

        @Override 
        public boolean isFinished() { 
            return true;
        }
    }

    public class Activate extends Command {

        private final HangSubsystem hangSubsystem;

        public Activate(HangSubsystem hangSubsystem){
            this.hangSubsystem = hangSubsystem;
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

        private final HangSubsystem hangSubsystem;

        public Reset(HangSubsystem hangSubsystem){
            this.hangSubsystem = hangSubsystem;
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
