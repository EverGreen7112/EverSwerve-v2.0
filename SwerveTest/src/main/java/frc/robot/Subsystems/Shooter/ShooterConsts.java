package frc.robot.Subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;
import frc.robot.Utils.Math.Vector3d;

public class ShooterConsts {
     //speaker
        public static final Vector3d RED_SPAKER_POS = new Vector3d(652.67 * 0.0254 + 0.05, 2.078, 218.42 * 0.0254 - 0.4);
        public static final Vector3d BLUE_SPAKER_POS = new Vector3d(1, 5.6, 2.2);
        public static final double SPEAKER_HEIGHT_SCALAR = 0.142354; 

        //gear ratio
        public static final double AIM_MOTOR_GEAR_RATIO = 1 / (45.0);

        //shooter speeds(in rpm)
        public static final double SPEAKER_SHOOT_SPEED = 6000;
        public static final double AMP_SHOOT_SPEED = 400;//1700
        public static final double CONTAINMENT_SPEED = 6000;
        public static final double SHOOT_SPEED_TOLERANCE = 320;
        
        // public static final double SHOOTER_AIM_MAX_VELOCITY = 260.0; // max RPM of shooter aim motor where its okay to shoot
        
        //shooter angles
        public static final double AIM_MOTOR_MAX_ANGLE = 180;
        public static final double AIM_MOTOR_MIN_ANGLE = -49.0;
        public static final double AIM_MOTOR_AMP_ANGLE = 98;//130 works on our amp not comp's
        public static final double AIM_MOTOR_SPEAKER_ANGLE = 0;
        public static final int AIM_MOTOR_CURRENT_LIMIT = 20;
        public static final double AIM_MOTOR_RATE_LIMIT = 1/9.0;
        public static final double AIM_MOTOR_SPEED_LIMIT = 0.2;
        public static final double AIM_MOTOR_MIN_SPEED = 0.0001;
        public static final double AIM_MOTOR_MIN_TOLERANCE = 3.0;

        //motor controllers ids
        public static final int RIGHT_SHOOT_MOTOR_ID = 6;
        public static final int LEFT_SHOOT_MOTOR_ID = 1;
        public static final int CONTAINMENT_MOTOR_ID = 7;
        public static final int AIM_MOTOR_ID = 9;
        public static final int NOTE_SENSOR_PORT = 3;

        //limit switches ids
        public static final int TOP_LIMIT_SWITCH_ID = 0;
        public static final int BOTTOM_LIMIT_SWITCH_ID = 9;

        //encoder
        public static final int EXTERNAL_AIM_MOTOR_ENCODER_ID = 0;//connected to the DIO

        //inverted motors
        public static final boolean RIGHT_SHOOT_MOTOR_INVERTED = false;
        public static final boolean LEFT_SHOOT_MOTOR_INVERTED = true;
        public static final boolean CONTAINMENT_MOTOR_INVERTED = false;
        public static final boolean AIM_MOTOR_INVERTED = true;
        
        //motors idle mode
        public static final IdleMode RIGHT_SHOOT_IDLE_MODE = IdleMode.kCoast;
        public static final IdleMode LEFT_SHOOT_IDLE_MODE = IdleMode.kCoast;
        public static final IdleMode CONTAINMENT_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode AIM_IDLE_MODE = IdleMode.kBrake;

        // physical constants
        public static final double SHOOTER_HEIGHT_METERS = 0.38287; // distance between the shooter axis of rotation to the ground

        public static final double EXTERNAL_ENCODER_OFFSET = 0.222401380560035;

        public static final double MIN_NOTE_DISTANCE = 20.0;//20.0

        //shooter angle
        public static final double SHOOTER_ANGLE_KP = 0.0075;//0.011
        public static final double SHOOTER_ANGLE_KI = 0.0000019; //0.0000003
        public static final double SHOOTER_ANGLE_KD = 0.0001;//0.03
        public static final double SHOOTER_ANGLE_KF = 0;//0.054 / 2;//0.057 / 2
        

}
