package frc.robot.Subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Math.Vector2d;


public class Shooter extends SubsystemBase{

    private static Shooter m_instance;
    //controls the right side shooter rollers
    private CANSparkMax m_rightShootMotor;
    //controls the left side shooter roller
    private CANSparkMax m_leftShootMotor;
    //controls the motor that puts note inside
    private CANSparkMax m_containmentMotor;
    //controls the angle of the shooter
    private CANSparkMax m_aimMotor;
    //external aim motor encoder
    public DutyCycleEncoder m_aimEncoder;
    //aim motor pid controller
    private PIDController m_aimPidController;
    //sends signal when the shooter is at the bottom angle
    private DigitalInput m_bottomLimitSwitch;
    //target angle
    private double m_targetAngle;
    //pid controller ff
    private double m_aimFF;
    //target shoot speed
    private double m_targetShootSpeed;
    // sensor to determine if note is in
    private AnalogInput m_noteSensor;


    private Shooter(){
        //create motor controller objects
        m_rightShootMotor = new CANSparkMax(ShooterConsts.RIGHT_SHOOT_MOTOR_ID, MotorType.kBrushless);
        m_leftShootMotor = new CANSparkMax(ShooterConsts.LEFT_SHOOT_MOTOR_ID, MotorType.kBrushless);
        m_containmentMotor = new CANSparkMax(ShooterConsts.CONTAINMENT_MOTOR_ID, MotorType.kBrushless);
        m_aimMotor = new CANSparkMax(ShooterConsts.AIM_MOTOR_ID, MotorType.kBrushless);
        
        //reset factory defaults
        m_rightShootMotor.restoreFactoryDefaults();
        m_leftShootMotor.restoreFactoryDefaults();
        m_containmentMotor.restoreFactoryDefaults();
        m_aimMotor.restoreFactoryDefaults();
        
        m_aimMotor.setSmartCurrentLimit(ShooterConsts.AIM_MOTOR_CURRENT_LIMIT);
        m_aimMotor.setOpenLoopRampRate(ShooterConsts.AIM_MOTOR_RATE_LIMIT);
        
        //set inverted
        m_rightShootMotor.setInverted(ShooterConsts.RIGHT_SHOOT_MOTOR_INVERTED);
        m_leftShootMotor.setInverted(ShooterConsts.LEFT_SHOOT_MOTOR_INVERTED);
        m_containmentMotor.setInverted(ShooterConsts.CONTAINMENT_MOTOR_INVERTED);
        m_aimMotor.setInverted(ShooterConsts.AIM_MOTOR_INVERTED);

        //set idle mode
        m_rightShootMotor.setIdleMode(ShooterConsts.RIGHT_SHOOT_IDLE_MODE);
        m_leftShootMotor.setIdleMode(ShooterConsts.LEFT_SHOOT_IDLE_MODE);
        m_containmentMotor.setIdleMode(ShooterConsts.CONTAINMENT_IDLE_MODE);
        m_aimMotor.setIdleMode(ShooterConsts.AIM_IDLE_MODE);
        
        //create external encoder instance
        m_aimEncoder = new DutyCycleEncoder(ShooterConsts.EXTERNAL_AIM_MOTOR_ENCODER_ID);
        m_aimEncoder.setPositionOffset(ShooterConsts.EXTERNAL_ENCODER_OFFSET);
        m_aimEncoder.setDistancePerRotation(-360);
        
        m_targetAngle = ShooterConsts.AIM_MOTOR_MIN_ANGLE;
        m_targetShootSpeed = 0;
        //create limit switch
        m_bottomLimitSwitch = new DigitalInput(ShooterConsts.BOTTOM_LIMIT_SWITCH_ID);
        //create pid controller
        m_aimPidController = new PIDController(ShooterConsts.SHOOTER_ANGLE_KP, ShooterConsts.SHOOTER_ANGLE_KI, ShooterConsts.SHOOTER_ANGLE_KD);
        m_noteSensor = new AnalogInput(ShooterConsts.NOTE_SENSOR_PORT);
    }

    /**
     * @return only instance of this class
     */
    public static Shooter getInstance(){
        if(m_instance == null)
            m_instance = new Shooter();
        return m_instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter angle", getShooterAngle());
        SmartDashboard.putNumber("target shooter angle", m_targetAngle);
        
        
        m_aimPidController.setSetpoint(m_targetAngle);
        //calculate current output
        double output = MathUtil.clamp(m_aimPidController.calculate(m_aimEncoder.getDistance()), -0.3, 0.3);//put this in consts i dont have time
        //activate motor with ff
        m_aimMotor.set(output + m_aimFF * Math.signum(output));
    }

    /**
    //  * calculate the angle the shooter needs to be at according to the current distance from the speaker
    //  * @return the angle the shooter needs to be at
    //  */
    // public double getShooterAngleToSpeaker(){
       
    //     Vector2d currentPos2d = SwerveLocalizer.getInstance().get
    //     //get position of shooter (NOT ROBOT)
    //     Vector3d currentPos = new Vector3d(currentPos2d.x, ShooterConsts.SHOOTER_HEIGHT_METERS, currentPos2d.y);
    //     Vector3d speakerPos = new Vector3d();
       
    //     //get position of speaker according to the alliance
    //     if(Robot.getAlliance() == Alliance.Blue){
    //         speakerPos = ShooterConsts.BLUE_SPAKER_POS;
    //     }
    //     else if(Robot.getAlliance() == Alliance.Red){
    //         speakerPos = ShooterConsts.RED_SPAKER_POS;
    //     }

    //     //calculate the vector between them
    //     Vector3d delta = currentPos.subtract(speakerPos);
    //     delta.add(new Vector3d(0, 
    //     -ShooterConsts.SPEAKER_HEIGHT_SCALAR * (delta.getX()*delta.getX() + delta.getZ()*delta.getZ()), 
    //     0));
    //     SmartDashboard.putNumber("distance from speaker",  Math.sqrt(delta.getX()*delta.getX() + delta.getZ()*delta.getZ()));
    //     return -Math.toDegrees(delta.getPitch());
    // }


    /**
     * Turn the shooter to given angle
     * @param angle - target shooter angle in degrees
     */
    public void turnToAngle(double angle){
        //clamp value to make sure bad input won't break the robot
        angle = MathUtil.clamp(angle, ShooterConsts.AIM_MOTOR_MIN_ANGLE, ShooterConsts.AIM_MOTOR_MAX_ANGLE);
        //reset accumalted I after changing target angles direction
        if(Math.signum(angle - getShooterAngle()) != Math.signum(m_targetAngle - getShooterAngle()))
            m_aimPidController.reset();

        m_targetAngle = angle;
        //scale kf according to the shooter's angle(watch the cosinus function graph inorder to understand why its here)
        m_aimFF = Math.cos(Math.toRadians(m_targetAngle)) * ShooterConsts.SHOOTER_ANGLE_KF;
        //set target angle
        m_aimPidController.setSetpoint(m_targetAngle);
    }

    public void pullNote(double speed){
        m_leftShootMotor.set(speed);
        m_rightShootMotor.set(speed);
        m_containmentMotor.set(speed);
    }

    public void chargeShoot(double speed){
        m_leftShootMotor.set(speed);
        m_rightShootMotor.set(speed);
    }

    public void pushNoteToShoot(double speed){
        m_containmentMotor.set(speed);
    }

    public double getTargetAngle(){
        return m_targetAngle;
    }

    /**
     * @return current shooter angle
     */
    public double getShooterAngle(){
        return m_aimEncoder.getDistance(); 
    }

    


    public void setContainmentSpeed(double speed){
        m_containmentMotor.set(speed);
    }

    /**
     * 
     * @return current speed of the right rollers
     */
    public double getRightRollersSpeed(){
        return m_rightShootMotor.getEncoder().getVelocity();
    }

    /**
     * 
     * @return current speed of the left roller
     */
    public double getLeftRollerSpeed(){
        return m_leftShootMotor.getEncoder().getVelocity();
    }

    /**
     * 
     * @return current speed of the containment roller
     */
    public double getContainmentSpeed(){
        return m_containmentMotor.getEncoder().getVelocity();
    }

    



}