package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallIntakeSubsystem extends SubsystemBase {

    // Instance
    private SparkMax m1_motor;
    private SparkMax m2_motor;

    public BallIntakeSubsystem() {
        // initialize motor
        m1_motor = new SparkMax(Constants.IntakeConstants.kBallMotorId1, MotorType.kBrushless);
        m2_motor = new SparkMax(Constants.IntakeConstants.kBallMotorId2, MotorType.kBrushless);
    }

    public void setTargetVelocity(double speed) {
        m1_motor.set(speed);
        m2_motor.set(-speed);
    }

    public boolean getHasBall(){
        return SensorSubsystem.getInstance().pipe;
    }

}
