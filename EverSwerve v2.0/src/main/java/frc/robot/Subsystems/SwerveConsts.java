package frc.robot.Subsystems;

import frc.robot.Utils.EverKit.EverAbsEncoder;
import frc.robot.Utils.EverKit.EverMotorController;
import frc.robot.Utils.EverKit.EverPIDController;
import frc.robot.Utils.EverKit.EverMotorController.IdleMode;
import frc.robot.Utils.EverKit.EverPIDController.ControlType;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverCANCoder;
import frc.robot.Utils.EverKit.Implementations.MotorControllers.EverSparkMax;
import frc.robot.Utils.EverKit.Implementations.PIDControllers.EverSparkMaxPIDController;
import frc.robot.Utils.Math.Vector2d;

/*
 * TL = Top Left
 * TR = Top Right
 * DL = Down Left
 * DR = Down Right
 */
public class SwerveConsts {
    public static final boolean USES_ABS_ENCODER = true;

    // speed values
    public static final double MAX_DRIVE_SPEED = 1.0; // m/s
    public static final double MAX_ANGULAR_SPEED = 1.0; // deg/s

    // motor controllers
    public static final EverSparkMax 
            TL_DRIVE_MOTOR = new EverSparkMax(0),
            TR_DRIVE_MOTOR = new EverSparkMax(0),
            DL_DRIVE_MOTOR = new EverSparkMax(0),
            DR_DRIVE_MOTOR = new EverSparkMax(0),
            TL_STEER_MOTOR = new EverSparkMax(0),
            TR_STEER_MOTOR = new EverSparkMax(0),
            DL_STEER_MOTOR = new EverSparkMax(0),
            DR_STEER_MOTOR = new EverSparkMax(0);

    public static final EverSparkMax[] DRIVE_MOTORS = {TL_DRIVE_MOTOR, TR_DRIVE_MOTOR, DL_DRIVE_MOTOR, DR_DRIVE_MOTOR};
    public static final EverSparkMax[] STEER_MOTORS = {TL_STEER_MOTOR, TR_STEER_MOTOR, DL_STEER_MOTOR, DR_STEER_MOTOR};

    // swerve module velocity pidf values
    public static final double WHEEL_VELOCITY_KP = 0.05, WHEEL_VELOCITY_KI = 0, WHEEL_VELOCITY_KD = 0,
            WHEEL_VELOCITY_KF = 0.75 / 2.81;
    // swerve module wheel angle pid values
    public static final double WHEEL_ANGLE_KP = 0.01, WHEEL_ANGLE_KI = 0.0, WHEEL_ANGLE_KD = 0.0;

    // swerve module pid controllers
    public static final EverSparkMaxPIDController 
            TL_VELOCITY_CONTROLLER = new EverSparkMaxPIDController(TL_DRIVE_MOTOR,
                    WHEEL_VELOCITY_KP, WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_KF),
            TR_VELOCITY_CONTROLLER = new EverSparkMaxPIDController(TR_DRIVE_MOTOR, WHEEL_VELOCITY_KP,
                    WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_KF),
            DL_VELOCITY_CONTROLLER = new EverSparkMaxPIDController(DL_DRIVE_MOTOR, WHEEL_VELOCITY_KP,
                    WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_KF),
            DR_VELOCITY_CONTROLLER = new EverSparkMaxPIDController(DR_DRIVE_MOTOR, WHEEL_VELOCITY_KP,
                    WHEEL_VELOCITY_KI, WHEEL_VELOCITY_KD, WHEEL_VELOCITY_KF);

    public static final EverSparkMaxPIDController[] WHEEL_VELOCITY_CONTROLLERS = {TL_VELOCITY_CONTROLLER, TR_VELOCITY_CONTROLLER, DL_VELOCITY_CONTROLLER, DR_VELOCITY_CONTROLLER};
    public static final EverSparkMaxPIDController                 
            TL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TL_STEER_MOTOR, WHEEL_ANGLE_KP, WHEEL_ANGLE_KI,
                    WHEEL_ANGLE_KD),
            TR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(TR_STEER_MOTOR, WHEEL_ANGLE_KP, WHEEL_ANGLE_KI,
                    WHEEL_ANGLE_KD),
            DL_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DL_STEER_MOTOR, WHEEL_ANGLE_KP, WHEEL_ANGLE_KI,
                    WHEEL_ANGLE_KD),
            DR_ANGLE_CONTROLLER = new EverSparkMaxPIDController(DR_STEER_MOTOR, WHEEL_ANGLE_KP,
                    WHEEL_ANGLE_KI, WHEEL_ANGLE_KD);

    public static final EverSparkMaxPIDController[] WHEEL_ANGLE_CONTROLLERS = {TL_ANGLE_CONTROLLER, TR_ANGLE_CONTROLLER, DL_ANGLE_CONTROLLER, DR_ANGLE_CONTROLLER};

    // swerve pid controllers
    public static final EverPIDController
            HEADING_ANGLE_CONTROLLER = null,
            X_CONTROLLER = null,
            Y_CONTROLLER = null;
    // chassis encoders
    public static final EverAbsEncoder
            TOP_LEFT_ABS_ENCODER = new EverCANCoder(0, 0),
            TOP_RIGHT_ABS_ENCODER = new EverCANCoder(0, 0),
            DOWN_LEFT_ABS_ENCODER = new EverCANCoder(0, 0),
            DOWN_RIGHT_ABS_ENCODER = new EverCANCoder(0, 0);
    // abs encoder offsets from their origin to the swerve modules's true origin
    public static final double TOP_RIGHT_ABS_ENCODER_OFFSET = 0, TOP_LEFT_ABS_ENCODER_OFFSET = 0,
            DOWN_RIGHT_ABS_ENCODER_OFFSET = 0, DOWN_LEFT_ABS_ENCODER_OFFSET = 0;

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
                new SwerveModule(TL_VELOCITY_CONTROLLER, TL_DRIVE_MOTOR, TL_DRIVE_MOTOR.getSparkInternalEncoder(), TL_ANGLE_CONTROLLER, TL_STEER_MOTOR, TL_STEER_MOTOR.getSparkInternalEncoder()),
                new SwerveModule(TR_VELOCITY_CONTROLLER, TR_DRIVE_MOTOR, TR_DRIVE_MOTOR.getSparkInternalEncoder(), TR_ANGLE_CONTROLLER, TR_STEER_MOTOR, TR_STEER_MOTOR.getSparkInternalEncoder()),
                new SwerveModule(DL_VELOCITY_CONTROLLER, DL_DRIVE_MOTOR, DL_DRIVE_MOTOR.getSparkInternalEncoder(), DL_ANGLE_CONTROLLER, DL_STEER_MOTOR, DL_STEER_MOTOR.getSparkInternalEncoder()),
                new SwerveModule(DR_VELOCITY_CONTROLLER, DR_DRIVE_MOTOR, DR_DRIVE_MOTOR.getSparkInternalEncoder(), DR_ANGLE_CONTROLLER, DR_STEER_MOTOR, DR_STEER_MOTOR.getSparkInternalEncoder())
     };

    public static void config() {
        for (EverSparkMax driveMotor : DRIVE_MOTORS) {
             driveMotor.restoreFactoryDefaults();
             driveMotor.setIdleMode(IdleMode.kCoast);
        }
        for (EverSparkMax steerMotor : STEER_MOTORS) {
             steerMotor.restoreFactoryDefaults();
             steerMotor.setIdleMode(IdleMode.kCoast);
        }
        for (EverSparkMaxPIDController velocityController : WHEEL_VELOCITY_CONTROLLERS) {
             velocityController.setConversionFactor(DRIVE_GEAR_RATIO * WHEEL_PERIMETER / 60.0, ControlType.kVel);
        }
        for (EverSparkMaxPIDController angleController : WHEEL_ANGLE_CONTROLLERS) {
             angleController.setConversionFactor(SwerveConsts.STEER_GEAR_RATIO * 360.0, ControlType.kPos);
        }
        
    }

}
