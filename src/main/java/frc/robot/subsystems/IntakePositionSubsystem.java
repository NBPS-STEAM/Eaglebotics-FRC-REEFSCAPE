package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Squid;

public class IntakePositionSubsystem extends SubsystemBase {

    public final SparkMax m_liftMotor1;
    public final SparkMax m_liftMotor2;
    public final RelativeEncoder m_liftEncoder;
    public final Squid m_liftPID;

    public final SparkMax m_pivotMotor1;
    public final AbsoluteEncoder m_pivotEncoder;
    public final Squid m_pivotPID;

    private double encoderValue;


    public IntakePositionSubsystem() {
        // Initialize PIDs
        m_liftPID = new Squid(Constants.IntakePositionConstants.kLiftP, Constants.IntakePositionConstants.kLiftI, Constants.IntakePositionConstants.kLiftD);
        m_liftPID.setTolerance(Constants.IntakePositionConstants.kLiftTolerance);
        m_liftPID.setSetpoint(Constants.IntakePositionConstants.stowLift);

        m_pivotPID = new Squid(Constants.IntakePositionConstants.kPivotP, Constants.IntakePositionConstants.kPivotI, Constants.IntakePositionConstants.kPivotD);
        m_pivotPID.setTolerance(Constants.IntakePositionConstants.kPivotTolerance);
        m_pivotPID.setSetpoint(Constants.IntakePositionConstants.stowPivot);

        // Lift motors
        m_liftMotor1 = new SparkMax(Constants.IntakePositionConstants.kLiftMotor1Id, MotorType.kBrushless);
        m_liftMotor2 = new SparkMax(Constants.IntakePositionConstants.kLiftMotor2Id, MotorType.kBrushless);
        SparkBaseConfig sharedConfig = new SparkMaxConfig().apply(Constants.kBrakeConfig).smartCurrentLimit(55, 55);
        SparkBaseConfig liftMotor1Config = new SparkMaxConfig().apply(sharedConfig).follow(m_liftMotor2);
        SparkBaseConfig liftMotor2Config = new SparkMaxConfig().apply(sharedConfig).inverted(true);
        m_liftMotor1.configure(liftMotor1Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_liftMotor2.configure(liftMotor2Config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_liftEncoder = m_liftMotor2.getAlternateEncoder();

        // Pivot motor
        m_pivotMotor1 = new SparkMax(Constants.IntakePositionConstants.kPivotMotor1Id, MotorType.kBrushless);
        m_pivotMotor1.configure(Constants.kBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_pivotEncoder = m_pivotMotor1.getAbsoluteEncoder();
    }

    public void setLiftSetpoint(double setpoint) {
        m_liftPID.setSetpoint(setpoint);
    }

    public void setPivotSetpoint(double setpoint) {
        m_pivotPID.setSetpoint(setpoint);
    }

    public boolean liftAtTargetPos(){
        return m_liftPID.atSetpoint();
    }

    public boolean pivotAtTargetPos(){
        return m_pivotPID.atSetpoint();
    }

    public boolean atTargetPos(){
        return liftAtTargetPos() && pivotAtTargetPos();
    }

    public void setIntakePositionSetpoints(double liftSetpoint, double pivotSetpoint) {
        setLiftSetpoint(liftSetpoint);
        setPivotSetpoint(pivotSetpoint);
    }

    @Override
    public void periodic() {
        encoderValue = m_liftPID.calculate(m_liftEncoder.getPosition());
        //m_liftMotor1.set(encoderValue); m_liftMotor1 follow m_liftMotor2
        m_liftMotor2.set(encoderValue);

        encoderValue = m_pivotPID.calculate(m_pivotEncoder.getPosition());
        m_pivotMotor1.set(encoderValue);
    }

}
