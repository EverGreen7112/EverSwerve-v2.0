package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;

public class Intake extends SubsystemBase{
    private static Intake m_instance = new Intake();
    private EverMotorController m_intakeMotor;

    private Intake(){
        m_intakeMotor = new EverSparkMax(19);
        m_intakeMotor.restoreFactoryDefaults();
    }

    public static Intake getInstance(){
        return m_instance;
    }

    public void intake(){
        m_intakeMotor.set(0.5);
    }

    public void eject(){
        m_intakeMotor.set(-0.5);
    }

    public void stop(){
        m_intakeMotor.set(0);
    }






    
}
