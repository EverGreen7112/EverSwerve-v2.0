package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;
import frc.robot.Utils.SensorsAndControllers.EverGyro;
import frc.robot.Utils.SensorsAndControllers.EverMotorController;

public class Swerve extends SubsystemBase {

    private SwerveModule[] m_modules;
    private EverGyro m_gyro;

    //driving vars
    private Vector2d m_driveVec;
    private boolean m_isGyroOriented;
    private PIDController m_headingController;
    private double m_headingTarget;
    private double m_rotationSpeed;

    private static Swerve m_instance;
    
    private Swerve() {
        //create modules
        if(SwerveConsts.USES_ABS_ENCODER){
            m_modules[0] = new SwerveModule(SwerveConsts.TOP_LEFT_DRIVE, SwerveConsts.TOP_LEFT_STEER, SwerveConsts.TOP_LEFT_ABS_ENCODER, SwerveConsts.TOP_LEFT_ABS_ENCODER_OFFSET);
            m_modules[1] = new SwerveModule(SwerveConsts.TOP_RIGHT_DRIVE, SwerveConsts.TOP_RIGHT_STEER, SwerveConsts.TOP_RIGHT_ABS_ENCODER, SwerveConsts.TOP_RIGHT_ABS_ENCODER_OFFSET);
            m_modules[2] = new SwerveModule(SwerveConsts.DOWN_LEFT_DRIVE, SwerveConsts.DOWN_LEFT_STEER, SwerveConsts.DOWN_LEFT_ABS_ENCODER, SwerveConsts.DOWN_LEFT_ABS_ENCODER_OFFSET);
            m_modules[3] = new SwerveModule(SwerveConsts.DOWN_RIGHT_DRIVE, SwerveConsts.DOWN_RIGHT_STEER, SwerveConsts.DOWN_RIGHT_ABS_ENCODER, SwerveConsts.DOWN_RIGHT_ABS_ENCODER_OFFSET);
        } 
        else{
            m_modules[0] = new SwerveModule(SwerveConsts.TOP_LEFT_DRIVE, SwerveConsts.TOP_LEFT_STEER);
            m_modules[1] = new SwerveModule(SwerveConsts.TOP_RIGHT_DRIVE, SwerveConsts.TOP_RIGHT_STEER);
            m_modules[2] = new SwerveModule(SwerveConsts.DOWN_LEFT_DRIVE, SwerveConsts.DOWN_LEFT_STEER);
            m_modules[3] = new SwerveModule(SwerveConsts.DOWN_RIGHT_DRIVE, SwerveConsts.DOWN_RIGHT_STEER);
        }
        
        m_gyro = SwerveConsts.GYRO;
        m_headingTarget = m_gyro.getYaw();
        m_headingController = new PIDController(SwerveConsts.HEADING_KP, SwerveConsts.HEADING_KI, SwerveConsts.HEADING_KD);
        m_rotationSpeed = 0;
        m_isGyroOriented = true;
    }

    /**
     * @return the only instance of the swerve
     */
    public static Swerve getInstance(){
        if(m_instance == null){
            m_instance = new Swerve();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        calcRotationSpeed();
        drive();
    }

    public void rotateTo(double angle){
        m_headingTarget = angle;
    }

    public void rotateBy(double angle){
        m_headingTarget += angle;
    }

    /**
     * see math on pdf document for more information
     * 
     * @param directionVec    - robot's target velocity
     * @param isGyroOriented - true for origin of the gyro relative driving
     *                         false for robot relative driving
     */
    public void drive(Vector2d driveVec, boolean isGyroOriented) {
       m_driveVec = driveVec;
       m_isGyroOriented = isGyroOriented;
    }

    private void drive(){
        // if drive values are 0 stop moving
        if (m_driveVec.mag() == 0 && m_rotationSpeed == 0) {
            for (int i = 0; i < m_modules.length; i++) {
                m_modules[i].stopModule();
            }
        }

        //clamp velocity
        if(m_driveVec.mag() > SwerveConsts.MAX_DRIVE_SPEED){
            m_driveVec.normalise();
            m_driveVec.mul(SwerveConsts.MAX_DRIVE_SPEED);
        }

        // convert driveVector to field oriented
        if(m_isGyroOriented) 
            m_driveVec.rotate(Math.toRadians(m_gyro.getYaw()));
        

        // calculate rotation vectors
        Vector2d[] rotVecs = new Vector2d[m_modules.length];
        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i] = new Vector2d(SwerveConsts.physicalMoudulesVector[i]);
            rotVecs[i].rotate(Math.toRadians(90));
            // change magnitude of rot vector to rotationSpeed
            rotVecs[i].normalise();
            rotVecs[i].mul(m_rotationSpeed);
        }

        Vector2d[] sumVectors = new Vector2d[m_modules.length];
        for (int i = 0; i < sumVectors.length; i++) {
            // sum rot and drive vectors
            sumVectors[i] = new Vector2d(m_driveVec);
            sumVectors[i].add(rotVecs[i]);

            // set module state
            m_modules[i].setState(sumVectors[i]);
        }
    }

    private void calcRotationSpeed(){
        //convert max angular speed to m/s from deg/s
        double ms_max_angular_speed = (SwerveConsts.MAX_ANGULAR_SPEED / 360.0) * SwerveConsts.ROBOT_BOUNDING_CIRCLE_PERIMETER;
        // get current angle
        double currentAngle = m_gyro.getYaw();
        // calculate optimized target angle
        double closestAngle = Funcs.closestAngle(currentAngle, m_headingTarget);
        double optimizedAngle = currentAngle + closestAngle;
        // get pid output
        m_rotationSpeed = MathUtil.clamp(m_headingController.calculate(currentAngle, optimizedAngle),
                -ms_max_angular_speed, ms_max_angular_speed);
    }

}
