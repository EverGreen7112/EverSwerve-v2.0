package frc.robot.Subsystems;

import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.Math.Vector2d;

public class SwerveLocalizator implements Periodic{

    private static SwerveLocalizator m_instance;
    private SwervePoint m_currentPoint;
    private double[] m_prevModulesDistance;
    private SwerveModule[] m_modules;

    private SwerveLocalizator(){
        m_modules = Swerve.getInstance().getModules();
        m_currentPoint = new SwervePoint(0, 0, Swerve.getInstance().getFieldOrientedAngle());
        m_prevModulesDistance = new double[m_modules.length];
        start(PeriodicTime.kRobotPeriodic);
    }

    public static SwerveLocalizator getInstance(){
        if(m_instance == null)
            m_instance = new SwerveLocalizator();
        return m_instance;
    }

    @Override
    public void periodic() {
        Vector2d robotDelta = getDelta();
        m_currentPoint.add(robotDelta.x, robotDelta.y);
        m_currentPoint.setAngle(Swerve.getInstance().getFieldOrientedAngle());
    }   

    public Vector2d getDelta(){
        Vector2d robotDelta = new Vector2d();
        for(int i = 0; i < m_prevModulesDistance.length; i++){
            double currentDistance = m_modules[i].getDistance();
            Vector2d deltaDistance = Vector2d.generateVec(currentDistance - m_prevModulesDistance[i], Math.toRadians(m_modules[i].getAngle()));
            robotDelta.add(deltaDistance);
            m_prevModulesDistance[i] = currentDistance;
        }
        robotDelta.mul(0.25);
        robotDelta.rotateBy(Math.toRadians(Swerve.getInstance().getFieldOrientedAngle()));
        return robotDelta;
    }


}