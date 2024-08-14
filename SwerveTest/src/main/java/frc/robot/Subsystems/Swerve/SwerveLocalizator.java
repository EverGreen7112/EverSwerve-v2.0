package frc.robot.Subsystems.Swerve;

import frc.robot.Subsystems.Vision.LocalizationVision;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.Math.Vector2d;

public class SwerveLocalizator implements Periodic {
    private int VISION_PORT = 5800;

    private LocalizationVision m_vision;
    private static SwerveLocalizator m_instance;
    private SwervePoint m_currentPoint;
    private double[] m_prevModulesDistance;
    private SwerveModule[] m_modules;
    private double m_prevVisionRobotX, m_prevVisionRobotY, m_prevVisionRobotAngle;

    private SwerveLocalizator() {
        m_modules = Swerve.getInstance().getModules();
        m_currentPoint = new SwervePoint(0, 0, Swerve.getInstance().getFieldOrientedAngle());
        m_prevModulesDistance = new double[m_modules.length];
        m_vision = new LocalizationVision(VISION_PORT);
        m_prevVisionRobotX = 0;
        m_prevVisionRobotY = 0;
        m_prevVisionRobotAngle = 0;
        start(PeriodicTime.kRobotPeriodic);
    }

    public static SwerveLocalizator getInstance() {
        if (m_instance == null)
            m_instance = new SwerveLocalizator();
        return m_instance;
    }

    @Override
    public void periodic() {

        //set the current point to the vision values
        SwervePoint currentVisionPoint = m_vision.get2DCords();
        if (currentVisionPoint.getX() != m_prevVisionRobotX ||
                currentVisionPoint.getY() != m_prevVisionRobotY ||
                currentVisionPoint.getAngle() != m_prevVisionRobotAngle) {

            m_currentPoint.set(currentVisionPoint.getX(), currentVisionPoint.getY(), currentVisionPoint.getAngle());
            m_prevVisionRobotX = currentVisionPoint.getX();
            m_prevVisionRobotY = currentVisionPoint.getY();
            m_prevVisionRobotAngle = currentVisionPoint.getAngle();
            restartOdometry();
        }

        //add odometry values to the current point
        Vector2d robotDelta = getDelta();
        m_currentPoint.add(robotDelta.x, robotDelta.y);
        m_currentPoint.setAngle(Swerve.getInstance().getFieldOrientedAngle());

    }

    /**
     * calculate odometry
     */
    public Vector2d getDelta() {
        Vector2d robotDelta = new Vector2d();
        for (int i = 0; i < m_prevModulesDistance.length; i++) {
            double currentDistance = m_modules[i].getDistance();
            Vector2d deltaDistance = Vector2d.generateVec(currentDistance - m_prevModulesDistance[i],
                    Math.toRadians(m_modules[i].getAngle()));
            robotDelta.add(deltaDistance);
            m_prevModulesDistance[i] = currentDistance;
        }
        robotDelta.mul(0.25);
        robotDelta.rotateBy(Math.toRadians(Swerve.getInstance().getFieldOrientedAngle()));
        return robotDelta;
    }

    public void restartOdometry() {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDistance(0);
            m_prevModulesDistance[i] = 0;
        }
    }

}