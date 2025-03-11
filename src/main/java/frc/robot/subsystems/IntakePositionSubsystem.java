package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

    private double liftClosedLoopReference = 0.0;
    private double pivotClosedLoopReference = 0.0;


    public IntakePositionSubsystem() {
        
        // Configure Lift Motors
        m_liftMotor1 = new SparkMax(IntakePositionConstants.kLiftMotor1Id, MotorType.kBrushless);
        m_liftMotor2 = new SparkMax(IntakePositionConstants.kLiftMotor2Id, MotorType.kBrushless);
        m_liftEncoder = m_liftMotor2.getAlternateEncoder();
        m_liftClosedLoopController = m_liftMotor2.getClosedLoopController();

        SparkBaseConfig sharedLiftConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(55, 55);
        SparkBaseConfig liftMotor1Config = new SparkMaxConfig().apply(sharedLiftConfig).follow(m_liftMotor2);
        SparkBaseConfig liftMotor2Config = new SparkMaxConfig().apply(sharedLiftConfig).inverted(true);
        liftMotor2Config.closedLoop.outputRange(-1, 1)
                                    .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                                    .pid(IntakePositionConstants.kLiftP, IntakePositionConstants.kLiftI, IntakePositionConstants.kLiftD)
                                    .iZone(IntakePositionConstants.kLiftIZone)
                                    .maxMotion.allowedClosedLoopError(IntakePositionConstants.kLiftTolerance);

        m_liftMotor1.configure(liftMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_liftMotor2.configure(liftMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure Pivot Motor
        m_pivotMotor1 = new SparkMax(IntakePositionConstants.kPivotMotor1Id, MotorType.kBrushless);
        m_pivotEncoder = m_pivotMotor1.getAbsoluteEncoder();
        m_pivotClosedLoopController = m_pivotMotor1.getClosedLoopController();

        SparkBaseConfig pivotMotor1Config = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(40, 40);
        pivotMotor1Config.closedLoop.outputRange(-1, 1)
                                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                    .pid(IntakePositionConstants.kPivotP, IntakePositionConstants.kPivotI, IntakePositionConstants.kPivotD)
                                    .iZone(IntakePositionConstants.kPivotIZone)
                                    .maxMotion.allowedClosedLoopError(IntakePositionConstants.kPivotTolerance);

        m_pivotMotor1.configure(pivotMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Go to Stow Position
        setIntakePositionSetpoints(IntakePositionConstants.stowLift, IntakePositionConstants.stowPivot);
    }

    public double getLiftPosition() {
        return m_liftEncoder.getPosition();
    }

    public double getLiftSetpoint() {
        return liftClosedLoopReference;
    }

    public void setLiftSetpoint(double setpoint) {
        liftClosedLoopReference = setpoint;
        m_liftClosedLoopController.setReference(liftClosedLoopReference, ControlType.kPosition);
    }

    public boolean liftAtTargetPos() {
        return Math.abs(getLiftSetpoint() - getLiftPosition()) < IntakePositionConstants.kLiftTolerance;
    }

    public double getPivotPosition() {
        return m_pivotEncoder.getPosition();
    }

    public double getPivotSetpoint() {
        return pivotClosedLoopReference;
    }

    public void setPivotSetpoint(double setpoint) {
        pivotClosedLoopReference = setpoint;
        m_pivotClosedLoopController.setReference(pivotClosedLoopReference, ControlType.kPosition);
    }

    public boolean pivotAtTargetPos() {
        return Math.abs(getPivotSetpoint() - getPivotPosition()) < IntakePositionConstants.kPivotTolerance;
    }

    public void setIntakePositionSetpoints(double liftSetpoint, double pivotSetpoint) {
        setLiftSetpoint(liftSetpoint);
        setPivotSetpoint(pivotSetpoint);
    }

    public boolean atTargetPos() {
        return liftAtTargetPos() && pivotAtTargetPos();
    }

}
