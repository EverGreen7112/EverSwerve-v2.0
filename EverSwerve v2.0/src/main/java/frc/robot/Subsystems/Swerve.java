package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

public class Swerve extends SubsystemBase {

    private SwerveModule[] m_modules;
    private AHRS m_gyro;    //TODO: change gyro to EverGyro

    //driving vars
    private Vector2d m_driveVec;
    private boolean m_isGyroOriented;
    private PIDController m_headingController; //TODO: change to EverPIDController(add EverExternalPIDController)
    private double m_headingTarget;
    private static Swerve m_instance;
    
    private Swerve() {
        SwerveConsts.config();
        m_modules = SwerveConsts.SWERVE_MODULES;
        m_gyro = new AHRS(SerialPort.Port.kMXP);
        m_headingController = new PIDController(0, 0, 0);
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
        drive();
        SmartDashboard.putNumber("tl", m_modules[0].getAngle());
        SmartDashboard.putNumber("tr", m_modules[1].getAngle());
        SmartDashboard.putNumber("dl", m_modules[2].getAngle());
        SmartDashboard.putNumber("dr", m_modules[3].getAngle());
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
        double desiredRotationSpeed = calcRotationSpeed();
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
            m_driveVec.rotate(Math.toRadians(m_gyro.getYaw()));
        

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
            sumVectors[i] = new Vector2d(m_driveVec);
            sumVectors[i].add(rotVecs[i]);

            // set module state
            m_modules[i].setState(sumVectors[i]);
        }
    }

    private double calcRotationSpeed(){
        //convert max angular speed to m/s from deg/s
        double ms_max_angular_speed = (SwerveConsts.MAX_ANGULAR_SPEED / 360.0) * SwerveConsts.ROBOT_BOUNDING_CIRCLE_PERIMETER;
        // get current angle
        double currentAngle = m_gyro.getYaw();
        // calculate optimized target angle
        double closestAngle = Funcs.closestAngle(currentAngle, m_headingTarget);
        double optimizedAngle = currentAngle + closestAngle;
        // return pid output
        return MathUtil.clamp(m_headingController.calculate(currentAngle, optimizedAngle),
                -ms_max_angular_speed, ms_max_angular_speed);
    }

}
