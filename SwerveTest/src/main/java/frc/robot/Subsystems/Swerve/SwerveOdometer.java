package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.Math.Vector2d;

public class SwerveOdometer {

    private static SwerveOdometer m_instance = new SwerveOdometer();

    private SwervePoint m_currentPoint; //position based only on odometry for callibration or debugging purposes
    private double[] m_prevModulesDistance;
    private SwerveModule[] m_modules;
    public SwerveOdometer(){
        m_currentPoint = new SwervePoint(0, 0, Swerve.getInstance().getGyroOrientedAngle());
        m_modules = Swerve.getInstance().getModules();
        m_prevModulesDistance = new double[m_modules.length];
    }

    public static SwerveOdometer getInstance(){
        return m_instance;
    }

    public SwervePoint getCurrentOdometryOnlyPoint(){
        return m_currentPoint;
    }

    /**
     * calculate odometry
     * returns delta since last call to the function
     *    
     */
    public Vector2d getDelta(double fieldOrientedAngle) {
        Vector2d robotDelta = new Vector2d();
        for (int i = 0; i < m_modules.length; i++) {
            double currentDistance = m_modules[i].getDistance();
            Vector2d deltaDistance = Vector2d.generateVec(currentDistance - m_prevModulesDistance[i],
                    Math.toRadians(m_modules[i].getAngle()));
            robotDelta.add(deltaDistance);
            m_prevModulesDistance[i] = currentDistance;
        }
        robotDelta.mul(1.0 / m_modules.length);
        
        //update position based only on odometry for callibration or debugging purposes
        m_currentPoint.add(robotDelta.x, robotDelta.y);
        m_currentPoint.setAngle(Swerve.getInstance().getGyroOrientedAngle());
        robotDelta.rotate(Math.toRadians(fieldOrientedAngle));
        return robotDelta;
    }
}
