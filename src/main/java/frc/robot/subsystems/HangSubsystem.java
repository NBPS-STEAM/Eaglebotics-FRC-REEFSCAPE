package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Servo;

public class HangSubsystem extends SubsystemBase {

    // Instance
    private SparkMax m1_motor;
    private SparkMax m2_motor;
    private final Servo servo1;
    private final Servo servo2;


    public HangSubsystem() {
        // initialize motor
        m1_motor = new SparkMax(Constants.HangConstants.kHangMotor1Id, MotorType.kBrushless);
        m2_motor = new SparkMax(Constants.HangConstants.kHangMotor2Id, MotorType.kBrushless);
        
        servo1 = new Servo(Constants.HangConstants.kServo1Channel);
        servo2 = new Servo(Constants.HangConstants.kServo2Channel);
    }

    public void setServoPositions(double position){
        if (position >= 0 && position <= 1.0){
            servo1.setPosition(position);
            servo2.setPosition(position);
        }
    }

    public void setTwistPower(double power) {
        m1_motor.set(power);
        m2_motor.set(power);

    }

}
