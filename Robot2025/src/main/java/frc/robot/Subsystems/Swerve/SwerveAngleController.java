package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Utils.EverKit.Periodic;

public class SwerveAngleController implements Periodic{

    private PIDController m_angleController;
    private double m_targetAngle;
    private boolean m_isFieldOriented;
    private static SwerveAngleController m_instance = new SwerveAngleController();

    private SwerveAngleController(){
        m_angleController = new PIDController(0.02, 0, 0);
        m_isFieldOriented = false;   
    }

    public static SwerveAngleController getInstance(){
        return m_instance;
    }

    public void start(double targetAngle){
        stop();
        start(PeriodicTime.kAutonomousPeriodic, PeriodicTime.kTeleopPeriodic, PeriodicTime.kTestPeriodic);
        m_targetAngle = targetAngle;
        m_isFieldOriented = false;
    }

    public void start(double targetAngle, boolean isFieldOriented){
        stop();
        start(PeriodicTime.kAutonomousPeriodic, PeriodicTime.kTeleopPeriodic, PeriodicTime.kTestPeriodic);
        m_targetAngle = targetAngle;
        m_isFieldOriented = isFieldOriented;
    }

    @Override
    public void periodic() {

        double currentAngle = (m_isFieldOriented) ? SwerveLocalizer.getInstance().getFieldOrientedAngle():
                                                    Swerve.getInstance().getGyroOrientedAngle();
        m_angleController.setSetpoint(m_targetAngle);
        double angularVelocity = m_angleController.calculate(currentAngle);
        Swerve.getInstance().driveByAngularVelocity(angularVelocity);   
    }

    public void stop(){
        stop(PeriodicTime.kAutonomousPeriodic, PeriodicTime.kTeleopPeriodic, PeriodicTime.kTestPeriodic);
    }
    
}
