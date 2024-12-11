package frc.robot.Subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverMotorControllerGroup;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;

public class Intake extends SubsystemBase{
    private final double SPEED = 0.6;
    private static Intake m_instance = new Intake();
    private EverMotorController m_intake;
    private EverMotorController m_leftShooterIntake;
    private EverMotorController m_rightShooterIntake;
    private EverMotorControllerGroup m_intakeMotorControllers;
    
    private Intake(){
        m_intake = new EverSparkMax(19);
        m_leftShooterIntake = new EverSparkMax(0);
        m_rightShooterIntake = new EverSparkMax(0);

        m_intakeMotorControllers = new EverMotorControllerGroup(m_intake, m_leftShooterIntake, m_rightShooterIntake);
        m_intakeMotorControllers.restoreFactoryDefaults();
    }

    public static Intake getInstance(){
        return m_instance;
    }

    public void intakeNote(){
        m_intakeMotorControllers.set(SPEED);
    }

    public void stop(){
        m_intakeMotorControllers.stop();
    }

    public void emitNote(){
        m_intakeMotorControllers.set(-SPEED);
    }
    
}