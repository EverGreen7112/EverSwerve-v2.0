package frc.robot.Subsystems;

import frc.robot.Utils.EverKit.Periodic;

public class SwerveOdometer implements Periodic{

    private static SwerveOdometer m_instance;

    private SwerveOdometer(){
        start(periodicTime.kRobotPeriodic);
    }

    public static SwerveOdometer getInstance(){
        if(m_instance == null){
            m_instance = new SwerveOdometer();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        
    }
    
}
