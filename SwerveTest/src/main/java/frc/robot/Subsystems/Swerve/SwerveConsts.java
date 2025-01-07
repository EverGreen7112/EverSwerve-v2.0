package frc.robot.Subsystems.Swerve;

import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverTalonFXInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverTalonFX;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverTalonFXPIDController;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

/*
 * TL = Top Left
 * TR = Top Right
 * DL = Down Left
 * DR = Down Right
 */
public interface SwerveConsts{
    // speed values
    public static final double MAX_NORMAL_DRIVE_SPEED = 2.8; // m/s
    public static final double MAX_TURBO_DRIVE_SPEED = 4;
    public static final double MAX_SLOW_DRIVE_SPEED = 1.5;
    public static final double MAX_ANGULAR_SPEED = 180; // deg/s/
    public static final double MIN_SPEED = 0.07;
    
    public static final double GYRO_DIRECTION = -1; //decide the direction of the gyro(counter clock wise should be positive)
    
    // motor controllers
    public static final EverTalonFX 
            TL_DRIVE_MOTOR = new EverTalonFX(6),
            TR_DRIVE_MOTOR = new EverTalonFX(13),
            DL_DRIVE_MOTOR = new EverTalonFX(9),
            DR_DRIVE_MOTOR = new EverTalonFX(10);
    
    public static final EverSparkMax 
            TL_STEER_MOTOR = new EverSparkMax(7),
            TR_STEER_MOTOR = new EverSparkMax(12),
            DL_STEER_MOTOR = new EverSparkMax(8),
            DR_STEER_MOTOR = new EverSparkMax(11);

    public static final EverTalonFX[] DRIVE_MOTORS = {TL_DRIVE_MOTOR, TR_DRIVE_MOTOR, DL_DRIVE_MOTOR, DR_DRIVE_MOTOR};
    public static final EverSparkMax[] STEER_MOTORS = {TL_STEER_MOTOR, TR_STEER_MOTOR, DL_STEER_MOTOR, DR_STEER_MOTOR};

    // encoders
    public static final EverTalonFXInternalEncoder 
            TL_DRIVE_ENCODER = new EverTalonFXInternalEncoder(TL_DRIVE_MOTOR),
            TR_DRIVE_ENCODER = new EverTalonFXInternalEncoder(TR_DRIVE_MOTOR),
            DL_DRIVE_ENCODER = new EverTalonFXInternalEncoder(DL_DRIVE_MOTOR),
            DR_DRIVE_ENCODER = new EverTalonFXInternalEncoder(DR_DRIVE_MOTOR);

    public static final EverSparkInternalEncoder 
            TL_STEER_ENCODER = new EverSparkInternalEncoder(TL_STEER_MOTOR),
            TR_STEER_ENCODER = new EverSparkInternalEncoder(TR_STEER_MOTOR),
            DL_STEER_ENCODER = new EverSparkInternalEncoder(DL_STEER_MOTOR),
            DR_STEER_ENCODER = new EverSparkInternalEncoder(DR_STEER_MOTOR);

    public static final EverTalonFXInternalEncoder[] DRIVE_ENCODERS = {TL_DRIVE_ENCODER, TR_DRIVE_ENCODER, DL_DRIVE_ENCODER, DR_DRIVE_ENCODER};
    public static final EverSparkInternalEncoder[] STEER_ENCODERS = {TL_STEER_ENCODER, TR_STEER_ENCODER, DL_STEER_ENCODER, DR_STEER_ENCODER};

    // swerve module pid controllers
    public static final EverTalonFXPIDController 
            TL_VELOCITY_CONTROLLER = new EverTalonFXPIDController(TL_DRIVE_MOTOR),
            TR_VELOCITY_CONTROLLER = new EverTalonFXPIDController(TR_DRIVE_MOTOR),
            DL_VELOCITY_CONTROLLER = new EverTalonFXPIDController(DL_DRIVE_MOTOR),
            DR_VELOCITY_CONTROLLER = new EverTalonFXPIDController(DR_DRIVE_MOTOR);

    public static final EverSparkMaxPIDController                 
            TL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TL_STEER_MOTOR),
            TR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TR_STEER_MOTOR),
            DL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DL_STEER_MOTOR),
            DR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DR_STEER_MOTOR);
    
    public static final EverTalonFXPIDController[] WHEEL_VELOCITY_CONTROLLERS = {TL_VELOCITY_CONTROLLER, TR_VELOCITY_CONTROLLER, DL_VELOCITY_CONTROLLER, DR_VELOCITY_CONTROLLER};
    public static final EverSparkMaxPIDController[] WHEEL_ANGLE_CONTROLLERS = {TL_ANGLE_CONTROLLER, TR_ANGLE_CONTROLLER, DL_ANGLE_CONTROLLER, DR_ANGLE_CONTROLLER};
            
    // chassis encoders 
    public static final EverAbsEncoder
            TL_ABS_ENCODER = new EverCANCoder(3),
            TR_ABS_ENCODER = new EverCANCoder(2),
            DL_ABS_ENCODER = new EverCANCoder(1),
            DR_ABS_ENCODER = new EverCANCoder(0);

    public static final EverAbsEncoder[] ABS_ENCODERS = {TL_ABS_ENCODER, TR_ABS_ENCODER, DL_ABS_ENCODER, DR_ABS_ENCODER};
    

    // swerve module velocity pidf values
    public static final double WHEEL_VELOCITY_KP = 0.0, WHEEL_VELOCITY_KI = 0.0, WHEEL_VELOCITY_KD = 0.00,
            WHEEL_VELOCITY_KF = 0.0;
    // swerve module wheel angle pid values
    public static final double WHEEL_ANGLE_KP = 0.01, WHEEL_ANGLE_KI = 0.0, WHEEL_ANGLE_KD = 0.0001;

    // swerve dimensions
    public static final double FRONT_WHEEL_DIST_METERS = 0.57, SIDE_WHEEL_DIST_METERS = 0.57;
    public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(
            FRONT_WHEEL_DIST_METERS * FRONT_WHEEL_DIST_METERS + SIDE_WHEEL_DIST_METERS * SIDE_WHEEL_DIST_METERS);
    public static final double ROBOT_RADIUS = 0.5 * Math.sqrt(
            FRONT_WHEEL_DIST_METERS * FRONT_WHEEL_DIST_METERS + SIDE_WHEEL_DIST_METERS * SIDE_WHEEL_DIST_METERS);
    public static final double WHEEL_PERIMETER = Math.PI * 0.095;

    // module gear ratios
    public static final double DRIVE_GEAR_RATIO = 1 / 5.9, STEER_GEAR_RATIO = 1 / 12.8;

    // swerve vectors
    public static final Vector2d 
            TR = new Vector2d((FRONT_WHEEL_DIST_METERS / 2),
                    (SIDE_WHEEL_DIST_METERS / 2)),
            TL = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2),
                    (SIDE_WHEEL_DIST_METERS / 2)),
            DR = new Vector2d(FRONT_WHEEL_DIST_METERS / 2,
                    -(SIDE_WHEEL_DIST_METERS / 2)),
            DL = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2),
                    -(SIDE_WHEEL_DIST_METERS / 2));

    // array of physical module vectors
    public static final Vector2d[] physicalMoudulesVector = { Funcs.convertFromStandardAxesToWpilibs(TL),
                                                              Funcs.convertFromStandardAxesToWpilibs(TR),
                                                              Funcs.convertFromStandardAxesToWpilibs(DL),
                                                              Funcs.convertFromStandardAxesToWpilibs(DR)
    };// array of vectors from robot center to swerves module

    
 

}
