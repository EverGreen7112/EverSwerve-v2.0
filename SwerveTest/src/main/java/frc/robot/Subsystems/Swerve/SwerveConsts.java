package frc.robot.Subsystems.Swerve;

import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.Math.Vector2d;

/*
 * TL = Top Left
 * TR = Top Right
 * DL = Down Left
 * DR = Down Right
 */
public interface SwerveConsts{
    public static final boolean USES_ABS_ENCODER = true;

    // speed values
    public static final double MAX_DRIVE_SPEED = 1.0; // m/s
    public static final double MAX_ANGULAR_SPEED = 1.0; // deg/s/

    // motor controllers
    public static final EverSparkMax 
            TL_DRIVE_MOTOR = new EverSparkMax(18),
            TR_DRIVE_MOTOR = new EverSparkMax(17),
            DL_DRIVE_MOTOR = new EverSparkMax(3),
            DR_DRIVE_MOTOR = new EverSparkMax(27),
            TL_STEER_MOTOR = new EverSparkMax(62),
            TR_STEER_MOTOR = new EverSparkMax(13),
            DL_STEER_MOTOR = new EverSparkMax(2),
            DR_STEER_MOTOR = new EverSparkMax(8);

    public static final EverSparkMax[] DRIVE_MOTORS = {TL_DRIVE_MOTOR, TR_DRIVE_MOTOR, DL_DRIVE_MOTOR, DR_DRIVE_MOTOR};
    public static final EverSparkMax[] STEER_MOTORS = {TL_STEER_MOTOR, TR_STEER_MOTOR, DL_STEER_MOTOR, DR_STEER_MOTOR};

    // swerve module velocity pidf values
    public static final double WHEEL_VELOCITY_KP = 0.05, WHEEL_VELOCITY_KI = 0, WHEEL_VELOCITY_KD = 0,
            WHEEL_VELOCITY_KF = 0.75 / 2.81;
    // swerve module wheel angle pid values
    public static final double WHEEL_ANGLE_KP = 0.01, WHEEL_ANGLE_KI = 0.0, WHEEL_ANGLE_KD = 0.0;

    // swerve module pid controllers
    public static final EverSparkMaxPIDController 
            TL_VELOCITY_CONTROLLER = new EverSparkMaxPIDController(TL_DRIVE_MOTOR),
            TR_VELOCITY_CONTROLLER = new EverSparkMaxPIDController(TR_DRIVE_MOTOR),
            DL_VELOCITY_CONTROLLER = new EverSparkMaxPIDController(DL_DRIVE_MOTOR),
            DR_VELOCITY_CONTROLLER = new EverSparkMaxPIDController(DR_DRIVE_MOTOR);

    public static final EverSparkMaxPIDController[] WHEEL_VELOCITY_CONTROLLERS = {TL_VELOCITY_CONTROLLER, TR_VELOCITY_CONTROLLER, DL_VELOCITY_CONTROLLER, DR_VELOCITY_CONTROLLER};
    public static final EverSparkMaxPIDController                 
            TL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TL_STEER_MOTOR),
            TR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TR_STEER_MOTOR),
            DL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DL_STEER_MOTOR),
            DR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DR_STEER_MOTOR);

    public static final EverSparkMaxPIDController[] WHEEL_ANGLE_CONTROLLERS = {TL_ANGLE_CONTROLLER, TR_ANGLE_CONTROLLER, DL_ANGLE_CONTROLLER, DR_ANGLE_CONTROLLER};

    // swerve pid controllers
    public static final EverPIDController
            HEADING_ANGLE_CONTROLLER = null,
            X_CONTROLLER = null,
            Y_CONTROLLER = null;
    // chassis encoders
    public static final EverAbsEncoder
            TL_ABS_ENCODER = new EverCANCoder(4),
            TR_ABS_ENCODER = new EverCANCoder(5),
            DL_ABS_ENCODER = new EverCANCoder(0),
            DR_ABS_ENCODER = new EverCANCoder(10);

    public static final EverAbsEncoder[] ABS_ENCODERS = {TL_ABS_ENCODER, TR_ABS_ENCODER, DL_ABS_ENCODER, DR_ABS_ENCODER};
    
    // swerve dimensions
    public static final double FRONT_WHEEL_DIST_METERS = 0.57, SIDE_WHEEL_DIST_METERS = 0.57;
    public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(
            FRONT_WHEEL_DIST_METERS * FRONT_WHEEL_DIST_METERS + SIDE_WHEEL_DIST_METERS * SIDE_WHEEL_DIST_METERS);
    public static final double WHEEL_PERIMETER = Math.PI * 0.095;

    // module gear ratios
    public static final double DRIVE_GEAR_RATIO = 1 / 6.75, STEER_GEAR_RATIO = 1 / 12.8;

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
    public static final Vector2d[] physicalMoudulesVector = { TL, TR, DL, DR};// array of vectors from robot center to swerves module

    public static final SwerveModule[] SWERVE_MODULES = {
        new SwerveModule(TL_VELOCITY_CONTROLLER, TL_DRIVE_MOTOR, new EverSparkInternalEncoder(TL_DRIVE_MOTOR), TL_ANGLE_CONTROLLER, TL_STEER_MOTOR, new EverSparkInternalEncoder(TL_STEER_MOTOR), ABS_ENCODERS[0]),
        new SwerveModule(TR_VELOCITY_CONTROLLER, TR_DRIVE_MOTOR, new EverSparkInternalEncoder(TR_DRIVE_MOTOR), TR_ANGLE_CONTROLLER, TR_STEER_MOTOR, new EverSparkInternalEncoder(TR_STEER_MOTOR), ABS_ENCODERS[1]),
        new SwerveModule(DL_VELOCITY_CONTROLLER, DL_DRIVE_MOTOR, new EverSparkInternalEncoder(DL_DRIVE_MOTOR), DL_ANGLE_CONTROLLER, DL_STEER_MOTOR, new EverSparkInternalEncoder(DL_STEER_MOTOR), ABS_ENCODERS[2]),
        new SwerveModule(DR_VELOCITY_CONTROLLER, DR_DRIVE_MOTOR, new EverSparkInternalEncoder(DR_DRIVE_MOTOR), DR_ANGLE_CONTROLLER, DR_STEER_MOTOR, new EverSparkInternalEncoder(DR_STEER_MOTOR), ABS_ENCODERS[3])
     };
 

}
