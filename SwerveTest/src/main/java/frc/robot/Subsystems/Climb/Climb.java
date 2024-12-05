package frc.robot.Subsystems.Climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final double SPEED = 0.4,
                    MAX_L_POS = 165, MAX_R_POS = 165;

    private static Climb m_instance = new Climb();
    private CANSparkMax m_motorL, m_motorR;
    private DigitalInput m_limitSwitchR, m_limitSwitchL;
   

    private Climb(){
        m_motorL = new CANSparkMax(61, MotorType.kBrushless);
        m_motorR = new CANSparkMax(15, MotorType.kBrushless);

        m_motorL.restoreFactoryDefaults();
        m_motorR.restoreFactoryDefaults();

        m_motorR.setInverted(true);

        m_limitSwitchL = new DigitalInput(2);
        m_limitSwitchR = new DigitalInput(1);
        

    }

    public static Climb getInstance(){
        return m_instance;
    }   


    public boolean isRightAtBottom(){
        return !m_limitSwitchR.get();
        
    }
    

    public boolean isLeftAtBottom(){
        return m_limitSwitchL.get();
    }

    public void extendRight(){
        if(m_motorR.getEncoder().getPosition() < MAX_R_POS)
            m_motorR.set(SPEED);
        else
            stopRight();
    }
    
    public void extendLeft(){
        if(m_motorL.getEncoder().getPosition() < MAX_L_POS)
            m_motorL.set(SPEED);
        else
            stopLeft();
    }

    public void retractLeft(){
        if(!isLeftAtBottom())
            m_motorL.set(-SPEED);
        else
            stopLeft();
    }

    public void retractRight(){
        if(!isRightAtBottom())
            m_motorR.set(-SPEED);
        else
            stopRight();
    }

    public void stopRight(){
        m_motorR.set(0);
    }

    public void stopLeft(){
        m_motorL.set(0);
    }

    @Override
    public void periodic(){
        //logs for debugging 
        SmartDashboard.putNumber("left climber pos", m_motorL.getEncoder().getPosition());
        SmartDashboard.putNumber("right climber pos", m_motorR.getEncoder().getPosition());
        SmartDashboard.putBoolean("left climber limitswitch", m_limitSwitchL.get());
        SmartDashboard.putBoolean("right climber limitswitch", !m_limitSwitchR.get());
    }
}
