package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PipeIntakeSubsystem extends SubsystemBase {

    // Instance
    public final SparkMax m_motor;

    public PipeIntakeSubsystem() {
        // initialize motor
        m_motor = new SparkMax(Constants.IntakeConstants.kPipeMotorId, MotorType.kBrushless);

        m_motor.configure(Constants.kBrakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTargetVelocity(double speed) {
        m_motor.set(speed);
    }
    
    public boolean getHasPipe(){
        return SensorSubsystem.getInstance().pipe;
    }

}
