package frc.robot.Subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class Swerve extends SubsystemBase implements SwerveConsts{

    private static Swerve m_instance = new Swerve();

    private SwerveModule[] m_modules;
    public AHRS m_gyro;    //TODO: change gyro to EverGyro

    //driving vars
    private Vector2d m_driveVec;
    private boolean m_isGyroOriented;
    private PIDController m_headingController;
    private double m_headingTarget;

    private Swerve() {
        m_driveVec = new Vector2d();

        //offset of abs encoder to 0 degrees being forward
        ABS_ENCODERS[0].setOffset(77.255859375/360.0);
        ABS_ENCODERS[1].setOffset(202.1484375/360.0);
        ABS_ENCODERS[2].setOffset(316.669921875/360.0);
        ABS_ENCODERS[3].setOffset(218.75978088378906/360.0);

        for (EverSparkMax driveMotor : DRIVE_MOTORS) {
             driveMotor.restoreFactoryDefaults();
             driveMotor.setInverted(false);
             driveMotor.setIdleMode(IdleMode.kCoast);
        }
        
        for (EverSparkMax steerMotor : STEER_MOTORS) {
             steerMotor.setIdleMode(IdleMode.kCoast);
        }
        
        for (EverSparkMaxPIDController velocityController : WHEEL_VELOCITY_CONTROLLERS) {
             velocityController.setPIDF(WHEEL_VELOCITY_KP, WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_KF);   
             velocityController.setConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER / 60.0, ControlType.kVel);
        }
        
        for (EverSparkMaxPIDController angleController : WHEEL_ANGLE_CONTROLLERS) {
             angleController.setPID(WHEEL_ANGLE_KP, WHEEL_ANGLE_KI, WHEEL_ANGLE_KD);      
             angleController.setConversionFactor(SwerveConsts.STEER_GEAR_RATIO * 360.0, ControlType.kPos);
        }

        m_modules = SwerveConsts.SWERVE_MODULES;
        m_gyro = new AHRS(SerialPort.Port.kMXP);
        m_gyro.reset();
        m_headingController = new PIDController(0.02675, 0, 0);
        m_headingController.setTolerance(0.05);
    }

    /**
     * @return the only instance of the swerve
     */
    public static Swerve getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("TL", m_modules[0].getAngle());
        SmartDashboard.putNumber("TR", m_modules[1].getAngle());
        SmartDashboard.putNumber("DL", m_modules[2].getAngle());
        SmartDashboard.putNumber("DR", m_modules[3].getAngle());

        // SmartDashboard.putNumber("angular velocity", getAngularVelocity());
        drive();
    }

    public double getGyroOrientedAngle(){
        return m_gyro.getAngle();
    }

    public SwerveModule[] getModules(){
        return m_modules;
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
     * @param driveVec    - robot's target velocity
     * @param isGyroOriented - true for origin of the gyro relative driving
     *                         false for robot relative driving
     */
    public void drive(Vector2d driveVec, boolean isGyroOriented) {
       m_driveVec = driveVec;
       m_isGyroOriented = isGyroOriented;
    }

    private void drive(){
        double desiredRotationSpeed = calcTargetRotationSpeed();
        // if drive values are 0 stop moving
        if (m_driveVec.mag() == 0 && desiredRotationSpeed == 0) {
            for (int i = 0; i < m_modules.length; i++) {
                m_modules[i].stopModule();
            }
        }

        //clamp velocity
        if(m_driveVec.mag() > SwerveConsts.MAX_DRIVE_SPEED){
            m_driveVec.normalise();
            m_driveVec.mul(SwerveConsts.MAX_DRIVE_SPEED);
        }

        // convert driveVector to gyro oriented
        if(m_isGyroOriented) 
            m_driveVec.rotate(Math.toRadians(getGyroOrientedAngle()));
        
        // calculate rotation vectors
        Vector2d[] rotVecs = new Vector2d[m_modules.length];
        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i] = new Vector2d(SwerveConsts.physicalMoudulesVector[i]);
            rotVecs[i].rotate(Math.toRadians(-90));
            // change magnitude of rot vector to rotationSpeed
            rotVecs[i].normalise();
            rotVecs[i].mul(desiredRotationSpeed);
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

    private double calcTargetRotationSpeed(){
        //convert max angular speed to m/s from deg/s
        double ms_max_angular_speed = (SwerveConsts.MAX_ANGULAR_SPEED / 360.0) * SwerveConsts.ROBOT_BOUNDING_CIRCLE_PERIMETER;
        // get current angle
        double currentAngle = getGyroOrientedAngle();
        // calculate optimized target angle
        double closestAngle = Funcs.closestAngle(currentAngle, m_headingTarget);
        double optimizedAngle = currentAngle + closestAngle;
        // return pid output
        return MathUtil.clamp(m_headingController.calculate(currentAngle, optimizedAngle),
                -ms_max_angular_speed, ms_max_angular_speed);
    }

    /** TODO: test this implementaion against one based on the gyro
     * get the swerves angular velocity (degrees / sec)
     */
    public double getAngularVelocity() {
        double angularVelocity = 0;
        
        for (int i = 0; i < m_modules.length; i++) {
            Vector2d moduleRotationVector = new Vector2d(SwerveConsts.physicalMoudulesVector[i]);
            moduleRotationVector.rotate(Math.toRadians(90));
            moduleRotationVector.normalise();

            Vector2d moduleVelocity = m_modules[i].getVelocity();

            angularVelocity += moduleVelocity.dot(moduleRotationVector);
        }
        
        // at this point the angular velocity is in m/s
        angularVelocity /= (double)SwerveConsts.physicalMoudulesVector.length;

        // converts angularVelocity to degrees/s
        angularVelocity /= SwerveConsts.ROBOT_BOUNDING_CIRCLE_PERIMETER;  // rotations / sec
        angularVelocity *= 360;  // degrees / sec

        return angularVelocity;
    }
    
    public Vector2d getVelocity(){
        Vector2d vel = new Vector2d();
        for(int i = 0; i < m_modules.length; i++){
            vel.add(m_modules[i].getVelocity());
        }
        vel.mul(1.0 / m_modules.length);
        return vel;
    }
    

}
