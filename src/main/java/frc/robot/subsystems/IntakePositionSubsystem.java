package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakePositionConstants;

public class IntakePositionSubsystem extends SubsystemBase {

    public final SparkMax m_liftMotor1;
    public final SparkMax m_liftMotor2;
    public final RelativeEncoder m_liftEncoder;
    public final SparkClosedLoopController m_liftClosedLoopController;

    public final SparkMax m_pivotMotor1;
    public final AbsoluteEncoder m_pivotEncoder;
    public final SparkClosedLoopController m_pivotClosedLoopController;

    private final SparkBaseConfig liftMotor1Config;
    private final SparkBaseConfig liftMotor2Config;
    private final SparkBaseConfig pivotMotor1Config;

    private double liftClosedLoopReference = 0.0;
    private double pivotClosedLoopReference = 0.0;

    private int positionLevel = 0;

    private boolean liftAscending = true;


    public IntakePositionSubsystem() {
        
        // Configure Lift Motors
        m_liftMotor1 = new SparkMax(IntakePositionConstants.kLiftMotor1Id, MotorType.kBrushless);
        m_liftMotor2 = new SparkMax(IntakePositionConstants.kLiftMotor2Id, MotorType.kBrushless);
        m_liftEncoder = m_liftMotor2.getAlternateEncoder();
        m_liftClosedLoopController = m_liftMotor2.getClosedLoopController();

        SparkBaseConfig sharedLiftConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(60, 60);
        liftMotor1Config = new SparkMaxConfig().apply(sharedLiftConfig).follow(m_liftMotor2);
        liftMotor2Config = new SparkMaxConfig().apply(sharedLiftConfig).inverted(true);
        liftMotor2Config.closedLoop.outputRange(-1, 1)
                                    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                                    .pid(IntakePositionConstants.kLiftPosP, IntakePositionConstants.kLiftI, IntakePositionConstants.kLiftPosD)
                                    .iZone(IntakePositionConstants.kLiftIZone)
                                    .maxMotion.allowedClosedLoopError(IntakePositionConstants.kLiftTolerance);

        m_liftMotor1.configure(liftMotor1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_liftMotor2.configure(liftMotor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // Configure Pivot Motor
        m_pivotMotor1 = new SparkMax(IntakePositionConstants.kPivotMotor1Id, MotorType.kBrushless);
        m_pivotEncoder = m_pivotMotor1.getAbsoluteEncoder();
        m_pivotClosedLoopController = m_pivotMotor1.getClosedLoopController();

        pivotMotor1Config = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(40, 40);
        pivotMotor1Config.closedLoop.outputRange(-1, 1)
                                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                    .pid(IntakePositionConstants.kPivotP, IntakePositionConstants.kPivotI, IntakePositionConstants.kPivotD)
                                    .iZone(IntakePositionConstants.kPivotIZone)
                                    .positionWrappingInputRange(0, 1.0)
                                    .positionWrappingEnabled(true)
                                    .maxMotion.allowedClosedLoopError(IntakePositionConstants.kPivotTolerance);

        m_pivotMotor1.configure(pivotMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Go to Stow Position
        setIntakePositionSetpoints(IntakePositionConstants.stowLift, IntakePositionConstants.stowPivot, 0);
    }

    public void updateAll() {
        // Change lift P when changing between ascending/descending
        boolean liftAscendingNow = getLiftError() >= 0;
        if (liftAscendingNow != liftAscending) {
            liftAscending = liftAscendingNow;
            if (liftAscending) {
                liftMotor2Config.closedLoop.pid(IntakePositionConstants.kLiftPosP, IntakePositionConstants.kLiftI, IntakePositionConstants.kLiftPosD);
            } else {
                liftMotor2Config.closedLoop.pid(IntakePositionConstants.kLiftNegP, IntakePositionConstants.kLiftI, IntakePositionConstants.kLiftNegD);
            }
            m_liftMotor2.configure(liftMotor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public void zeroLift() {
        m_liftEncoder.setPosition(0);
    }

    public double getLiftPosition() {
        return m_liftEncoder.getPosition();
    }

    public double getLiftSetpoint() {
        return liftClosedLoopReference;
    }

    public void setLiftSetpoint(double setpoint, Integer forLevel) {
        if (forLevel != null) positionLevel = forLevel;
        liftClosedLoopReference = setpoint;
        m_liftClosedLoopController.setReference(liftClosedLoopReference, ControlType.kPosition, ClosedLoopSlot.kSlot0, IntakePositionConstants.kLiftAntigrav, ArbFFUnits.kPercentOut);
        //m_liftClosedLoopController.setReference(liftClosedLoopReference, ControlType.kPosition);
    }

    public void offsetLiftSetpoint(double delta) {
        setLiftSetpoint(liftClosedLoopReference + delta, null);
    }

    public void setLiftSpeed(double speed) {
        m_liftMotor2.set(speed);
    }

    public double getLiftError() {
        return getLiftSetpoint() - getLiftPosition();
    }

    public boolean liftAtTargetPos() {
        return Math.abs(getLiftError()) < IntakePositionConstants.kLiftTolerance;
    }

    public double getPivotPosition() {
        return m_pivotEncoder.getPosition();
    }

    public double getPivotSetpoint() {
        return pivotClosedLoopReference;
    }

    public void setPivotSetpoint(double setpoint, Integer forLevel) {
        if (forLevel != null) positionLevel = forLevel;
        pivotClosedLoopReference = setpoint;
        m_pivotClosedLoopController.setReference(pivotClosedLoopReference, ControlType.kPosition);
    }

    public void offsetPivotSetpoint(double delta) {
        setPivotSetpoint(pivotClosedLoopReference + delta, null);
    }

    public void setPivotSpeed(double speed) {
        m_pivotMotor1.set(speed);
    }

    public double getPivotError() {
        return getPivotSetpoint() - getPivotPosition();
    }

    public boolean pivotAtTargetPos() {
        return Math.abs(getPivotError()) < IntakePositionConstants.kPivotTolerance;
    }

    public void setIntakePositionSetpoints(double liftSetpoint, double pivotSetpoint, Integer forLevel) {
        setLiftSetpoint(liftSetpoint, forLevel);
        setPivotSetpoint(pivotSetpoint, forLevel);
    }

    public int getPositionLevel() {
        return positionLevel;
    }

    public boolean atTargetPos() {
        return liftAtTargetPos() && pivotAtTargetPos();
    }
    public double[] getCurrent(){
        double[] out={m_liftMotor1.getOutputCurrent(),m_liftMotor2.getOutputCurrent(),m_pivotMotor1.getOutputCurrent()};
        return out;
    }

}
