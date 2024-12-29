package frc.robot.Subsystems.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.Subsystems.Vision.LocalizationVision;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.Math.Vector2d;

public class SwerveLocalizerV2 implements Periodic {

    private static SwerveLocalizerV2 m_instance = new SwerveLocalizerV2();

    private SwerveDrivePoseEstimator m_poseEstimator;
    private Supplier<Double> m_gyroAngle;
    private SwerveModulePosition[] m_modulesPosition = new SwerveModulePosition[4];
    private Pose2d m_initialStartPos;

    private static SwerveDriveKinematics m_swerveKinematics;

    private LocalizationVision m_vision;
    private final int VISION_PORT = 5800;
    private final float VISION_FRAME_TIME = 1.0f / 20.0f;

    public SwerveLocalizerV2(){
        
        m_swerveKinematics = new SwerveDriveKinematics(SwerveConsts.TOP_LEFT_SWERVE_MODULE_LOCATION, SwerveConsts.TOP_RIGHT_SWERVE_MODULE_LOCATION,
        SwerveConsts.DOWN_LEFT_SWERVE_MODULE_LOCATION, SwerveConsts.DOWN_RIGHT_SWERVE_MODULE_LOCATION);

        m_gyroAngle = () -> Swerve.getInstance().getGyroOrientedAngle();

        m_modulesPosition[0] = new SwerveModulePosition(Swerve.getInstance().getModules()[0].getDistance(),Rotation2d.fromDegrees(Swerve.getInstance().getModules()[0].getAngle()));
        m_modulesPosition[1] = new SwerveModulePosition(Swerve.getInstance().getModules()[1].getDistance(),Rotation2d.fromDegrees(Swerve.getInstance().getModules()[1].getAngle()));
        m_modulesPosition[2] = new SwerveModulePosition(Swerve.getInstance().getModules()[2].getDistance(),Rotation2d.fromDegrees(Swerve.getInstance().getModules()[2].getAngle()));
        m_modulesPosition[3] = new SwerveModulePosition(Swerve.getInstance().getModules()[3].getDistance(),Rotation2d.fromDegrees(Swerve.getInstance().getModules()[3].getAngle()));

        m_initialStartPos = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

        m_poseEstimator = new SwerveDrivePoseEstimator(m_swerveKinematics, Rotation2d.fromDegrees(m_gyroAngle.get()), m_modulesPosition, m_initialStartPos);

        m_vision = new LocalizationVision(VISION_PORT);
        m_vision.setOnNewPointReceived((SwervePoint newPoint) ->{
            Pose2d visionPos = new Pose2d(newPoint.getX(), newPoint.getY(), Rotation2d.fromDegrees(newPoint.getAngle()));
            m_poseEstimator.addVisionMeasurement(visionPos, VISION_FRAME_TIME);

        });
        start(PeriodicTime.kRobotPeriodic);
    }
    

    public static SwerveLocalizerV2 getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    public Pose2d getPosition(){
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPosition(Pose2d pos){
        // m_poseEstimator.getEstimatedPosition().getTranslation().minus(m_poseEstimator.getEstimatedPosition().getTranslation());
        // m_poseEstimator.getEstimatedPosition().getTranslation().plus(pos.getTranslation());
        // m_poseEstimator.getEstimatedPosition().getRotation().minus(m_poseEstimator.getEstimatedPosition().getRotation());
        // m_poseEstimator.getEstimatedPosition().getRotation().plus(pos.getRotation());
        m_poseEstimator.resetPosition(null, m_modulesPosition, pos);
    }

    public double getFieldOrientedAngle(){
        return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    public double getAngleToSpeaker(){
        Vector2d speakerPos;
        if(Robot.getAlliance() == Alliance.Red){
            speakerPos = new Vector2d(16.54, 5.8928 - 0.6);
        }
        else{
            speakerPos = new Vector2d(0,  5.8928 - 0.4);
        }
        
        Vector2d currentPos = new Vector2d(m_poseEstimator.getEstimatedPosition().getTranslation().getX(), m_poseEstimator.getEstimatedPosition().getTranslation().getY());
        Vector2d deltaToSpeaker = speakerPos.subtract(currentPos.x, currentPos.y);
        return Math.toDegrees(deltaToSpeaker.theta());
    }

      public void updateOdometry() {
        m_poseEstimator.update(
            Rotation2d.fromDegrees(m_gyroAngle.get()),
            new SwerveModulePosition[] {
                new SwerveModulePosition(Swerve.getInstance().getModules()[0].getDistance(),Rotation2d.fromDegrees(Swerve.getInstance().getModules()[0].getAngle())),
                new SwerveModulePosition(Swerve.getInstance().getModules()[0].getDistance(),Rotation2d.fromDegrees(Swerve.getInstance().getModules()[0].getAngle())),          
                new SwerveModulePosition(Swerve.getInstance().getModules()[0].getDistance(),Rotation2d.fromDegrees(Swerve.getInstance().getModules()[0].getAngle())),  
                new SwerveModulePosition(Swerve.getInstance().getModules()[0].getDistance(),Rotation2d.fromDegrees(Swerve.getInstance().getModules()[0].getAngle()))
            });

        // Also apply vision measurements. We use 0.3 seconds in the past as an example -- on
        // a real robot, this must be calculated based either on latency or timestamps.
        // m_poseEstimator.addVisionMeasurement(
        //     ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(
        //         m_poseEstimator.getEstimatedPosition()),
        //     Timer.getTimestamp() - 0.3);
  }
    
}
