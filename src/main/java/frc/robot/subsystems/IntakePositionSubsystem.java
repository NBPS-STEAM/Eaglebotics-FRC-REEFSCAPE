package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.Squid;

public class IntakePositionSubsystem extends SubsystemBase {

    private SparkMax m_liftMotor1;
    private SparkMax m_liftMotor2;
    private Encoder m_liftEncoder=new Encoder(0,1);
    private Squid m_liftPID;

    private SparkMax m_pivotMotor1;
    private AbsoluteEncoder m_pivotEncoder;
    private Squid m_pivotPID;

    private double encoderValue;


    public IntakePositionSubsystem() {
        m_liftMotor1 = new SparkMax(Constants.IntakePositionConstants.kLiftMotor1Id, MotorType.kBrushless);
        m_liftMotor2 = new SparkMax(Constants.IntakePositionConstants.kLiftMotor2Id, MotorType.kBrushless);
        m_liftPID = new Squid(Constants.IntakePositionConstants.kLiftP, Constants.IntakePositionConstants.kLiftI, Constants.IntakePositionConstants.kLiftD);
        m_liftPID.setTolerance(5);
        m_pivotMotor1 = new SparkMax(Constants.IntakePositionConstants.kPivotMotor1Id, MotorType.kBrushless);
        m_pivotEncoder = m_pivotMotor1.getAbsoluteEncoder();
        m_pivotPID = new Squid(Constants.IntakePositionConstants.kPivotP, Constants.IntakePositionConstants.kPivotI, Constants.IntakePositionConstants.kPivotD);
        m_pivotPID.setTolerance(5);
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
        return m_liftPID.atSetpoint()&&pivotAtTargetPos();
    }

    public void setIntakePositionSetpoints(double liftSetpoint, double pivotSetpoint) {
        setLiftSetpoint(liftSetpoint);
        setPivotSetpoint(pivotSetpoint);
    }

    @Override
    public void periodic() {
        encoderValue = (m_liftPID.calculate(m_liftEncoder.getDistance()));
        m_liftMotor1.set(encoderValue);
        m_liftMotor2.set(encoderValue);

        encoderValue = (m_pivotPID.calculate(m_pivotEncoder.getPosition()));
        m_pivotMotor1.set(encoderValue);
        }

}