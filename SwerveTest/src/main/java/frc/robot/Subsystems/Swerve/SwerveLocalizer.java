package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Vision.LocalizationVision;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

/**
    NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
 */
public class SwerveLocalizer implements Periodic {
  
    private final float VISION_FRAME_TIME = 1.0f / 20.0f;
    private final int VISION_PORT = 5800;
    private final double MIN_DIFF_FOR_ANGLE_OFFSET_REPLACEMENT = 10.0; // minimum difference between the new calculated angle offset and the old angle offset to replace the old offset by the new offset (degrees)
    private final double ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT = 0.5; // how much weight do we give the new offset when averaging with the last offset
    // (theoretically) the lower the number the less offset drifting
    // keep in mind this number should be between 0 and 1 with 0 meaning that we'll always stick to the old offset and 1 meaning we'll always replace the offset not caring about the old offset
    // also keep in mind that even if you set it to 0 the offset will still update if the new offset is drastically different than the last

    private static SwerveLocalizer m_instance = new SwerveLocalizer();

    private SwerveOdometer m_odometer;
    private LocalizationVision m_vision;

    private SwervePoint m_currentPoint;
    private double m_angleOffsetToField; //the offset between the gyro angle to the field angle
    
    //custom offset for when setting robot's odometry values before the match begin using the setCurrentPoint function 
    private double m_offsetNotFromVision;
    private boolean m_isVisionWorking;
    private SwerveLocalizer() {

        Vector2d startPos = new Vector2d(
                                        SwerveConsts.FRONT_WHEEL_DIST_METERS / 2.0,
                                        SwerveConsts.SIDE_WHEEL_DIST_METERS / 2.0);

        m_currentPoint = new SwervePoint(startPos.x,
                                         startPos.y,
                                         Swerve.getInstance().getGyroOrientedAngle());
        m_odometer = SwerveOdometer.getInstance();
        m_vision = new LocalizationVision(VISION_PORT);
        m_angleOffsetToField = 0;
        m_isVisionWorking = false;
        m_offsetNotFromVision = 0;

        m_vision.setOnNewPointReceived((SwervePoint newPoint) -> {
            m_isVisionWorking = true;
            //set the current point to the vision values
            
            //update position
            SwervePoint currentVisionPoint = newPoint;
            m_currentPoint.set(currentVisionPoint.getX(), currentVisionPoint.getY());
            
            //update angle
            double angularVelocity = Swerve.getInstance().getAngularVelocity();
            double newAngle = currentVisionPoint.getAngle() + (angularVelocity * VISION_FRAME_TIME) + 90.0;
            /*
            * we add an estimation for delta angle to the angle given by the vision
            * this is an attempt to compensate for the fact that the data as given by the
            * vision is delayed
            * this was added to help compensate for angle offset drifting
            */
            double newOffset = newAngle - Swerve.getInstance().getGyroOrientedAngle();
            // if the new angle offset is drastically different than the last, we should try 
            if(Math.abs(m_angleOffsetToField - newOffset) > MIN_DIFF_FOR_ANGLE_OFFSET_REPLACEMENT){
                m_angleOffsetToField = newOffset;
            } 
            else{
                // this averages all angle offset values over time but giving more weight to more recent values
                // this is supposed to help stop offset drfiting, but worse case scenario it still slows offset drifting by ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT
                m_angleOffsetToField = (ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT * newOffset) +
                    ((1 - ANGLE_OFFSET_AVERAGE_NEW_READING_WEIGHT) * m_angleOffsetToField);
            }
            
            m_currentPoint.setAngle(getFieldOrientedAngle());
        });
        start(PeriodicTime.kRobotPeriodic);
    }

    public static SwerveLocalizer getInstance() {
        return m_instance;
    }

    @Override
    public void periodic() {
        //add odometry values to the current point
        Vector2d robotDelta = m_odometer.getDelta(getFieldOrientedAngle());
        m_currentPoint.add(robotDelta.x, robotDelta.y);
        m_currentPoint.setAngle(getFieldOrientedAngle());
    }

    /**
    * returns point in WPIlib's coordinate system
    * NWU - positive X is forward positive Y is left positive rotation is counter-clock wise
    */
    public SwervePoint getCurrentPoint(){
        return m_currentPoint;
    }

    public void setCurrentPoint(SwervePoint newPoint){
        m_offsetNotFromVision = newPoint.getAngle() - Swerve.getInstance().getGyroOrientedAngle();
        m_currentPoint = new SwervePoint(newPoint);
    }

    public double getFieldOrientedAngle(){
        return Swerve.getInstance().getGyroOrientedAngle() + ((m_isVisionWorking) ? m_angleOffsetToField : m_offsetNotFromVision);
    }
}