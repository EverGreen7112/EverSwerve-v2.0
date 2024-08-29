package frc.robot.Subsystems;

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
    private AHRS m_gyro;    //TODO: change gyro to EverGyro

    //driving vars
    private Vector2d m_tVelocity;
    private boolean m_isGyroOriented;
    private double m_tAngularVelocity;
    private PIDController m_headingController;
    

    private Swerve() {
        m_tVelocity = new Vector2d();
        m_tAngularVelocity = 0;

        //offset of abs encoder to 0 degrees being forward
        ABS_ENCODERS[0].setOffset(166.904296875/360.0);
        ABS_ENCODERS[1].setOffset(292.236328125/360.0);
        ABS_ENCODERS[2].setOffset(47.8125/360.0);
        ABS_ENCODERS[3].setOffset(308.056640625/360.0);

        for (EverSparkMax driveMotor : DRIVE_MOTORS) {
             driveMotor.restoreFactoryDefaults();
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
        m_headingController.setTolerance(0.5);
    }

    /**
     * @return the only instance of the swerve
     */
    public static Swerve getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("angular velocity", getAngularVelocity());

        drive();
    }

    public double getGyroOrientedAngle(){
        return m_gyro.getAngle();
    }

    public SwerveModule[] getModules(){
        return m_modules;
    }

   
    public void drive(Vector2d velocity, boolean isGyroOriented, double angularVelocity) {
       m_tVelocity = velocity;
       m_isGyroOriented = isGyroOriented;
       m_tAngularVelocity = angularVelocity;
    }

    private void drive(){
        
        double desiredRotationSpeed = m_headingController.calculate(getAngularVelocity(), m_tAngularVelocity);
        
        // if drive values are 0 stop moving
        if (m_tVelocity.mag() == 0 && desiredRotationSpeed == 0) {
            for (int i = 0; i < m_modules.length; i++) {
                m_modules[i].stopModule();
            }
        }

        //clamp velocity
        if(m_tVelocity.mag() > SwerveConsts.MAX_DRIVE_SPEED){
            m_tVelocity.normalise();
            m_tVelocity.mul(SwerveConsts.MAX_DRIVE_SPEED);
        }

        // convert driveVector to gyro oriented
        if(m_isGyroOriented) 
            m_tVelocity.rotate(Math.toRadians(m_gyro.getYaw()));
        
        // calculate rotation vectors
        Vector2d[] rotVecs = new Vector2d[m_modules.length];
        for (int i = 0; i < rotVecs.length; i++) {
            rotVecs[i] = new Vector2d(SwerveConsts.physicalMoudulesVector[i]);
            rotVecs[i].rotate(Math.toRadians(90));
            // change magnitude of rot vector to rotationSpeed
            rotVecs[i].normalise();
            rotVecs[i].mul(desiredRotationSpeed);
        }

        Vector2d[] sumVectors = new Vector2d[m_modules.length];
        for (int i = 0; i < sumVectors.length; i++) {
            // sum rot and drive vectors
            sumVectors[i] = new Vector2d(m_tVelocity);
            sumVectors[i].add(rotVecs[i]);

            // set module state
            m_modules[i].setState(sumVectors[i]);
        }
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

            double moduleVel = m_modules[i].getSpeed();
            //get current speed in each axis
            double moduleX = (Math.cos(Math.toRadians(m_modules[i].getAngle())) * moduleVel);
            double moduleY = (Math.sin(Math.toRadians(m_modules[i].getAngle())) * moduleVel);
            Vector2d moduleVelocity = new Vector2d(moduleX, moduleY);

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
