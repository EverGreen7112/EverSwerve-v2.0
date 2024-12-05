package frc.robot.Subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final double SPEED = 0.6;
    private static Intake m_instance = new Intake();
    private CANSparkMax m_motor;

    private Intake(){
        m_motor = new CANSparkMax(19, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
    }

    public static Intake getInstance(){
        return m_instance;
    }

    public void intakeNote(){
        m_motor.set(SPEED);
    }

    public void stop(){
        m_motor.set(0);
    }

    public void emitNote(){
        m_motor.set(-SPEED);
    }
    
}