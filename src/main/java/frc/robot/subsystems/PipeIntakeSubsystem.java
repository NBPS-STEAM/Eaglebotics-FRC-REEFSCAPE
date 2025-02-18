package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PipeIntakeSubsystem extends SubsystemBase {

    // Instance
    private SparkMax m_motor;
    private DigitalInput pipeSwitch=new DigitalInput(2);

    public PipeIntakeSubsystem() {
        // initialize motor
        m_motor = new SparkMax(Constants.IntakeConstants.kPipeMotorId, MotorType.kBrushless);
    }

    public void setTargetVelocity(double speed) {
        m_motor.set(speed);
    }
    public boolean getHasPipe(){
        return pipeSwitch.get();
    }

}
