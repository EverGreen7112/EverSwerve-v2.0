package frc.robot.Subsystems.LocalizationTest;
                                                                        
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Utils.EverKit.Periodic;

public class LocalizationTest implements Periodic, SwerveConsts{
    
    private static final LocalizationCamera[] CAMS = { 
        new LocalizationCamera(null, null, null, null, null),
        new LocalizationCamera(null, null, null, null, null)
    }; 

    private static final double FIELD_WIDTH = 0;
    private static final double FIELD_HEIGHT = 0;
    private static final double MAX_CAMERA_HEIGHT = 0;

    
    private static LocalizationTest m_instance = new LocalizationTest();
    private ArrayList<LocalizationCamera> m_cams;
    private SwerveDrivePoseEstimator m_poseEstimator;

    private LocalizationTest(){
        m_cams = new ArrayList<>();
       
        
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(modulesPositions[0].x, modulesPositions[0].y),
                new Translation2d(modulesPositions[1].x, modulesPositions[1].y),
                new Translation2d(modulesPositions[2].x, modulesPositions[2].y),
                new Translation2d(modulesPositions[3].x, modulesPositions[3].y));


        m_poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                Swerve.getInstance().getGyroRotation(),
                Swerve.getInstance().getModulesPositions(),
                new Pose2d());


    }

    public static LocalizationTest getInstance(){
        return m_instance;
    }
    @Override
    public void periodic() {
        
        // update odometry
        m_poseEstimator.update(Swerve.getInstance().getGyroRotation(), Swerve.getInstance().getModulesPositions());
        
    }

    /*
     * returns point in WPIlib's coordinate system
     * NWU - positive X is forward positive Y is left positive rotation is
     * counter-clock wise
     */
    public Pose2d getCurrentPoint() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setCurrentPoint(Pose2d newPoint) {
        m_poseEstimator.resetPosition(
                Swerve.getInstance().getGyroRotation(),
                Swerve.getInstance().getModulesPositions(),
                newPoint);
    }

    public double getFieldOrientedAngle() {
        return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }
    

    private boolean takeVisionPoseEstimation(Optional<EstimatedRobotPose> est){
        if(!est.isPresent() || est == null)
            return false;
        
        Pose2d estPos = est.get().estimatedPose.toPose2d();
        int targetsUsed = est.get().targetsUsed.size();
        double x = estPos.getX();
        double y = estPos.getY();
        double z = est.get().estimatedPose.getZ();

        boolean outOfField = x < 0.0 || x > FIELD_WIDTH || y < 0.0 || y > FIELD_HEIGHT;
        boolean aboveCamera = z > MAX_CAMERA_HEIGHT; 
        boolean underGround = z < 0; 
        boolean noTags = targetsUsed == 0;

        // Cannot be high ambiguity
        boolean isHighAmbiguity =
                (targetsUsed == 1 && est.get()..ambiguity() > maxAmbiguity);

        // Check whether to reject pose
        return notEmpty || isHighAmbiguity || onGround || inFieldBounds;
        
    
    }

    private void setVisionMeasurements(Optional<EstimatedRobotPose> est){
        if(!takeVisionPoseEstimation(est))
            return;
        m_poseEstimator.addVisionMeasurement(est.get().estimatedPose.toPose2d(), est.get().timestampSeconds);
        

    }

    
}
