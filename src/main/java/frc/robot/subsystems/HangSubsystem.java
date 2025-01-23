package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {

    // Instance
    private SparkMax m1_motor;
    private RelativeEncoder m1_encoder;
    private SparkMax m2_motor;
    private PIDController m_pidController;

    public HangSubsystem() {
        // initialize motor
        m1_motor = new SparkMax(Constants.HangConstants.kHangMotor1Id, MotorType.kBrushless);
        m1_encoder = m1_motor.getEncoder();

        m2_motor = new SparkMax(Constants.HangConstants.kHangMotor2Id, MotorType.kBrushless);

        m_pidController = new PIDController(Constants.HangConstants.kHangP, Constants.HangConstants.kHangI, Constants.HangConstants.kHangD);
        m_pidController.setIZone(Constants.HangConstants.kHangIz);
    }

    public void setTwistPosition(double position) {
        m_pidController.setSetpoint(position);
    }

    public boolean isAtTwistPosition() {
        return m_pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        double speed = m_pidController.calculate(m1_encoder.getPosition());
        m1_motor.set(speed);
        m2_motor.set(-speed);
    }

}
