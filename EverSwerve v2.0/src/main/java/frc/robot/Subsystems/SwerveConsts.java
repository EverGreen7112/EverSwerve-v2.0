package frc.robot.Subsystems;

import frc.robot.Utils.Math.Vector2d;
import frc.robot.Utils.SensorsAndControllers.EverAbsEncoder;
import frc.robot.Utils.SensorsAndControllers.EverGyro;
import frc.robot.Utils.SensorsAndControllers.EverMotorController;
import frc.robot.Utils.SensorsAndControllers.Implementations.MotorControllers.EverSpark;

public class SwerveConsts{

    public static final boolean USES_ABS_ENCODER = true;

    // speed values
    public static final double MAX_DRIVE_SPEED = 1.0;   //m/s
    public static final double MAX_ANGULAR_SPEED = 1.0; //deg/s

    // motor controllers
    public static final EverMotorController TOP_LEFT_DRIVE = new EverSpark(0);
    public static final EverMotorController TOP_RIGHT_DRIVE = new EverSpark(0);
    public static final EverMotorController DOWN_LEFT_DRIVE = new EverSpark(0);
    public static final EverMotorController DOWN_RIGHT_DRIVE = new EverSpark(0);
    public static final EverMotorController TOP_LEFT_STEER = new EverSpark(0);
    public static final EverMotorController TOP_RIGHT_STEER = new EverSpark(0);
    public static final EverMotorController DOWN_LEFT_STEER = new EverSpark(0);
    public static final EverMotorController DOWN_RIGHT_STEER = new EverSpark(0);

    // chassis encoders
    public static final EverAbsEncoder TOP_LEFT_ABS_ENCODER = null;
    public static final EverAbsEncoder TOP_RIGHT_ABS_ENCODER = null;
    public static final EverAbsEncoder DOWN_LEFT_ABS_ENCODER = null;
    public static final EverAbsEncoder DOWN_RIGHT_ABS_ENCODER = null;

    // abs encoder offsets from their origin to the swerve modules's true origin
    public static final double TOP_RIGHT_ABS_ENCODER_OFFSET = 0;
    public static final double TOP_LEFT_ABS_ENCODER_OFFSET = 0;
    public static final double DOWN_RIGHT_ABS_ENCODER_OFFSET = 0;
    public static final double DOWN_LEFT_ABS_ENCODER_OFFSET = 0;
 
    //gyro
    public static final EverGyro GYRO = null;

    //sizes
    public static final double FRONT_WHEEL_DIST_METERS = 0.57;
    public static final double SIDE_WHEEL_DIST_METERS = 0.57;
    public static final double WHEEL_PERIMETER = Math.PI * 0.095;
    public static final double ROBOT_BOUNDING_CIRCLE_PERIMETER = Math.PI * Math.sqrt(
        FRONT_WHEEL_DIST_METERS * FRONT_WHEEL_DIST_METERS + SIDE_WHEEL_DIST_METERS * SIDE_WHEEL_DIST_METERS);

    // swerve vectors
    public static final Vector2d TOP_RIGHT = new Vector2d((FRONT_WHEEL_DIST_METERS / 2),
            (SIDE_WHEEL_DIST_METERS / 2));
    public static final Vector2d TOP_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2),
            SIDE_WHEEL_DIST_METERS / 2);
    public static final Vector2d DOWN_RIGHT = new Vector2d(FRONT_WHEEL_DIST_METERS / 2,
            -(SIDE_WHEEL_DIST_METERS / 2));
    public static final Vector2d DOWN_LEFT = new Vector2d(-(FRONT_WHEEL_DIST_METERS / 2),
            -(SIDE_WHEEL_DIST_METERS / 2));

    // array of physical module vectors
    public static final Vector2d[] physicalMoudulesVector = { TOP_RIGHT, TOP_LEFT,
            DOWN_RIGHT, DOWN_LEFT };// array of vectors from robot center to swerves module

    // module gear ratios
    public static final double DRIVE_GEAR_RATIO = 1 / 6.75;
    public static final double STEER_GEAR_RATIO = 1 / 12.8;

    // swerve module wheel angle pid values
    public static final double WHEEL_ANGLE_KP = 0.01;
    public static final double WHEEL_ANGLE_KI = 0.0;
    public static final double WHEEL_ANGLE_KD = 0.0;

    // swerve module velocity pid values
    public static final double WHEEL_VELOCITY_KP = 0.05;
    public static final double WHEEL_VELOCITY_KI = 0;
    public static final double WHEEL_VELOCITY_KD = 0;
    public static final double WHEEL_VELOCITY_KF = 0.75 / 2.81;

    // pos pid values
    public static final double POS_KP = 0;
    public static final double POS_KI = 0;//5
    public static final double POS_KD = 0;//2
    public static final double X_TOLERANCE = 0.05;
    public static final double Y_TOLERANCE = 0.05;

    // heading pid values
    public static final double HEADING_KP = 0.03;
    public static final double HEADING_KI = 0;
    public static final double HEADING_KD = 0;
    public static final double HEADING_TOLERANCE = 3;


    
}
