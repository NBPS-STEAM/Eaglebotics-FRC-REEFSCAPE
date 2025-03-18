package frc.robot.commands.oldordrivecommands.ScoreCommands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePositionSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public final class IntakePositionCommand {

    //Constructor to allow for each Command to reference the subsystem without a parameter
    private IntakePositionSubsystem intakePositionSubsystem;

    public IntakePositionCommand(IntakePositionSubsystem intakePositionSubsystem) {
        this.intakePositionSubsystem = intakePositionSubsystem;
    }
    


    //Sets the elevator and the intake arm to a specified position, which is then reached through PIDs
    public class SetIntakePositionSetpoints extends Command {

        private double liftSetpoint;
        private double pivotSetpoint;
        private Integer forLevel;

        public SetIntakePositionSetpoints(double liftSetpoint, double pivotSetpoint, Integer forLevel) {
            this.liftSetpoint = liftSetpoint;
            this.pivotSetpoint = pivotSetpoint;
            this.forLevel = forLevel;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setIntakePositionSetpoints(liftSetpoint, pivotSetpoint, forLevel);
            if(liftSetpoint!=Constants.IntakePositionConstants.stowLift)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.atTargetPos();
        }
    }



    //Individual setter for the elevator
    public class SetLiftSetpoint extends Command {

        private double liftSetpoint;
        private Integer forLevel;

        public SetLiftSetpoint(double liftSetpoint, Integer forLevel) {
            this.liftSetpoint = liftSetpoint;
            this.forLevel = forLevel;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setLiftSetpoint(liftSetpoint, forLevel);
            if(liftSetpoint!=Constants.IntakePositionConstants.stowLift)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.liftAtTargetPos();
        }
    }



    //Individual setter for the intake arm
    public class SetPivotSetpoint extends Command {

        private double pivotSetpoint;
        private Integer forLevel;

        public SetPivotSetpoint(double pivotSetpoint, Integer forLevel) {
            this.pivotSetpoint = pivotSetpoint;
            this.forLevel = forLevel;
            addRequirements(intakePositionSubsystem);
        }

        
        @Override
        public void initialize() {
            intakePositionSubsystem.setPivotSetpoint(pivotSetpoint, forLevel);
            if(pivotSetpoint!=Constants.IntakePositionConstants.stowPivot)LEDSubsystem.getInstance().setPlacePos();
        }

        @Override
        public boolean isFinished() {
            return intakePositionSubsystem.pivotAtTargetPos();
        }
    }



    /**
     * Adjust the lift's position by running it at velocity for duration of command.
     * Continuously updates a DoubleConsumer as it runs with the current position.
     * Sets the lift's setpoint to current position when the command ends.
     */
    public class AdjustLift extends Command {

        private DoubleSupplier rateSupplier;
        private DoubleConsumer positionConsumer;

        public AdjustLift(DoubleSupplier rateSupplier, DoubleConsumer positionConsumer) {
            this.rateSupplier = rateSupplier;
            this.positionConsumer = positionConsumer;
            addRequirements(intakePositionSubsystem);
        }

        @Override
        public void execute() {
            intakePositionSubsystem.setLiftSpeed(rateSupplier.getAsDouble());
            updateConsumer();
        }

        @Override
        public void end(boolean interrupted) {
            updateConsumer();
            intakePositionSubsystem.setLiftSetpoint(intakePositionSubsystem.getLiftPosition(), null); // also cancels setting velocity
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        private void updateConsumer() {
            if (positionConsumer != null) positionConsumer.accept(intakePositionSubsystem.getLiftPosition());
        }
    }



    /**
     * Adjust the pivot's position by offsetting its position for duration of command.
     * Continuously updates a DoubleConsumer as it runs with the current position.
     * Sets the pivot's setpoint to current position when the command ends.
     */
    public class AdjustPivot extends Command {

        private DoubleSupplier rateSupplier;
        private DoubleConsumer positionConsumer;

        public AdjustPivot(DoubleSupplier rateSupplier, DoubleConsumer positionConsumer) {
            this.rateSupplier = rateSupplier;
            this.positionConsumer = positionConsumer;
            addRequirements(intakePositionSubsystem);
        }

        @Override
        public void execute() {
            intakePositionSubsystem.setPivotSpeed(rateSupplier.getAsDouble());
            updateConsumer();
        }

        @Override
        public void end(boolean interrupted) {
            updateConsumer();
            intakePositionSubsystem.setPivotSetpoint(intakePositionSubsystem.getPivotPosition(), null); // also cancels setting velocity
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        private void updateConsumer() {
            if (positionConsumer != null) positionConsumer.accept(intakePositionSubsystem.getPivotPosition());
        }
    }

}