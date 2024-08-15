package frc.robot.Subsystems.Swerve;

import frc.robot.Subsystems.Vision.LocalizationVision;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.Math.Vector2d;

public class SwerveLocalizator implements Periodic {
    private int VISION_PORT = 5800;
    private double MIN_DIFF_FOR_ANGLE_OFFSET_REPLACEMENT = 0;
    private double ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT = 0;

    private static SwerveLocalizator m_instance;

    private SwervePoint m_currentPoint;
    private LocalizationVision m_vision;
    private double m_angleOffsetToField; //the offset between the gyro angle to the field angle
    private double[] m_prevModulesDistance;
    private SwerveModule[] m_modules;
    private double m_prevVisionRobotX, m_prevVisionRobotY, m_prevVisionRobotAngle;

    private SwerveLocalizator() {
        m_modules = Swerve.getInstance().getModules();
        m_currentPoint = new SwervePoint(0, 0, Swerve.getInstance().getGyroOrientedAngle());
        m_prevModulesDistance = new double[m_modules.length];
        m_vision = new LocalizationVision(VISION_PORT);
        m_prevVisionRobotX = 0;
        m_prevVisionRobotY = 0;
        m_prevVisionRobotAngle = 0;
        m_angleOffsetToField = 0;
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
        if (currentVisionPoint.getX() != m_prevVisionRobotX || currentVisionPoint.getY() != m_prevVisionRobotY) {
            
            //update position
            m_currentPoint.set(currentVisionPoint.getX(), currentVisionPoint.getY());
            m_prevVisionRobotX = currentVisionPoint.getX();
            m_prevVisionRobotY = currentVisionPoint.getY();
            restartOdometry();
        }
        
        //update angle
        if(currentVisionPoint.getAngle() != m_prevVisionRobotAngle){
            // if the new angle offset is drastically different than the last, we should try 
            /*if (Math.abs(m_angleOffset - new_offset) > ChassisValues.MIN_DIFF_FOR_ANGLE_OFFSET_REPLACEMENT){
                m_angleOffset = new_offset;
            } else {
            // this averages all angle offset values over time but giving more weight to more recent values
            // this is supposed to help stop offset drfiting, but worse case scenario it still slows offset drifting by ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT
            m_angleOffset = (ChassisValues.ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT * new_offset) +
             ((1 - ChassisValues.ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT) * m_angleOffset); 
            }*/
            double newOffset = currentVisionPoint.getAngle() - Swerve.getInstance().getGyroOrientedAngle();
            if(Math.abs(m_angleOffsetToField - newOffset) > MIN_DIFF_FOR_ANGLE_OFFSET_REPLACEMENT){
                m_angleOffsetToField = newOffset;
            }
            else{
                 // this averages all angle offset values over time but giving more weight to more recent values
                // this is supposed to help stop offset drfiting, but worse case scenario it still slows offset drifting by ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT
                m_angleOffsetToField = (ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT * newOffset) +
                    ((1 - ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT) * m_angleOffsetToField);
            }

        }

        //add odometry values to the current point
        Vector2d robotDelta = getDelta();
        m_currentPoint.add(robotDelta.x, robotDelta.y);

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
        robotDelta.rotateBy(Math.toRadians(getFieldOrientedAngle()));
        return robotDelta;
    }

    public void restartOdometry() {
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDistance(0);
            m_prevModulesDistance[i] = 0;
        }
    }

    public SwervePoint getCurrentPoint(){
        return m_currentPoint;
    }

    public double getFieldOrientedAngle(){
        return Swerve.getInstance().getGyroOrientedAngle() + m_angleOffsetToField;
    }
}