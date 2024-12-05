package frc.robot.Subsystems.Shooter;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Subsystems.Swerve.SwervePoint;
import frc.robot.Utils.Math.Funcs;

public class Shooter extends SubsystemBase {
    private final double 

                         INTAKE_SPEED = 0.5, CONTAINMENT_SPEED = 0.5, SHOOT_SPEED = 1, 
                         MIN_ANGLE = -49, MAX_ANGLE = 130,
                         BLUE_SPEAKER_X = 0, BLUE_SPEAKER_Y = 5.8928 + (0.01 * (15 * 2.54)),
                         RED_SPEAKER_X = 16.54, RED_SPEAKER_Y = 5.8928 + (0.01 * (15 * 2.54)),
                         SPEAKER_H = 3.5;

    private static Shooter m_instance = new Shooter();
    public CANSparkMax m_pivotMotor, m_leftShoot, m_rightShoot, m_containmentMotor;
    private Supplier <Double> m_pivotAngle;
    private DutyCycleEncoder m_pivotEncoder;
    private PIDController m_angleController;
    private double m_targetAngle;

    private Shooter(){
        m_leftShoot = new CANSparkMax(1, MotorType.kBrushless);
        m_rightShoot = new CANSparkMax(6, MotorType.kBrushless);
        m_leftShoot.restoreFactoryDefaults();
        m_rightShoot.restoreFactoryDefaults();
        m_leftShoot.setInverted(true);

        m_pivotMotor = new CANSparkMax(9, MotorType.kBrushless);
        m_containmentMotor = new CANSparkMax(7, MotorType.kBrushless);
        m_pivotMotor.restoreFactoryDefaults();
        m_containmentMotor.restoreFactoryDefaults();
        m_containmentMotor.setInverted(true);
        m_pivotMotor.setIdleMode(IdleMode.kBrake);
        m_pivotEncoder = new DutyCycleEncoder(0);
        m_pivotEncoder.setDistancePerRotation(-1 * 360); //set conversion ratio 
        m_pivotEncoder.setPositionOffset(0.222401380560035);

        m_angleController = new PIDController(0.0045,  0.0000015, 0.00018);

        m_pivotAngle = () -> {return m_pivotEncoder.getDistance();};
        m_angleController.setTolerance(3);
        m_targetAngle = MIN_ANGLE;

    }

    

    @Override
    public void periodic(){
        
        m_angleController.setSetpoint(m_targetAngle);
        m_pivotMotor.set(-1 * MathUtil.clamp(m_angleController.calculate(m_pivotAngle.get()), -0.25, 0.25));
        
    }

    public static Shooter getInstance(){
        return m_instance;
    }

    public void containNote(){
        m_containmentMotor.set(CONTAINMENT_SPEED);
    }

    public void stopContain(){
        m_containmentMotor.stopMotor();
    }

    public void releaseNote(){
        m_containmentMotor.set(-CONTAINMENT_SPEED);
    }

    public void shoot(){
        m_rightShoot.set(SHOOT_SPEED);
        m_leftShoot.set(SHOOT_SPEED);
    }

    public void shoot(double speedL, double speedR){
        m_rightShoot.set(speedL);
        m_leftShoot.set(speedR);
    }

    public void shootToAmp(){
        m_rightShoot.set(0.15);
        m_leftShoot.set(0.15);
    }

    public void stopShoot(){
        m_rightShoot.stopMotor();
        m_leftShoot.stopMotor();
    }
    
    public void intakeNote(){
        m_leftShoot.set(-INTAKE_SPEED);
        m_rightShoot.set(-INTAKE_SPEED);
        m_containmentMotor.set(CONTAINMENT_SPEED);
    }

    public void stopIntake(){
        m_leftShoot.stopMotor();
        m_rightShoot.stopMotor();
        m_containmentMotor.stopMotor();
    }

    public void emitNote(){
        m_leftShoot.set(INTAKE_SPEED);
        m_rightShoot.set(INTAKE_SPEED);
        m_containmentMotor.set(-CONTAINMENT_SPEED);
    }

    public void stopEmission(){
        m_leftShoot.stopMotor();
        m_rightShoot.stopMotor();
        m_containmentMotor.stopMotor();
    }

    public void turnTo(double angle){
        m_angleController.reset();//reset the I
        m_targetAngle = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);        
    }

    public void turnToIntake(){
        turnTo(MIN_ANGLE);
    }

    public void turnToAmp(){
        turnTo(98);
    }

    public boolean readyToIntake(){
        return MathUtil.isNear(MIN_ANGLE, m_pivotAngle.get(), 2); //is the shooter at the bottom
    }

    public void autoAim(){
        double targetAngle, distance;
        SwervePoint pos = SwerveLocalizer.getInstance().getCurrentPoint();
        if(Robot.getAlliance() == Alliance.Blue){
            distance = Math.sqrt(Math.pow(pos.getX() - BLUE_SPEAKER_X,2) + Math.pow(pos.getY() - BLUE_SPEAKER_Y,2));
        }
        else{
            distance = Math.sqrt(Math.pow(RED_SPEAKER_X - pos.getX(),2) + Math.pow(RED_SPEAKER_Y - pos.getY(),2));
        }
        

        targetAngle = Math.toDegrees(Math.atan2(SPEAKER_H , distance));
        turnTo(targetAngle);
        
    }


}
