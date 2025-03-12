package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private final Spark m_Blinken;

    private static LEDSubsystem instance;

    public static LEDSubsystem getInstance(){//singleton because it makes it easier to use, can be switched if kelin wants
        if (instance == null) {
            instance = new LEDSubsystem();
        }
        return instance;
    }

    private LEDSubsystem(){
        m_Blinken= new Spark(5);
    }
    public void setStow(){
        m_Blinken.set(0.85);//dark blue
    }
    public void setPlacePos(){
        m_Blinken.set(0.67);//gold
    }
    public void setOuttake(){
        m_Blinken.set(0.07);//flashes whatever we set color 1 to fast
    }
    public void setIntake(){
        m_Blinken.set(0.27);//flashes whatever we set color 2 to fast
    }
    
}
