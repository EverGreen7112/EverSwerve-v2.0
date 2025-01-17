// package frc.robot.Subsystems.LocalizationTest;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;


//      public static final String CAM_NAME = "YOUR CAMERA NAME";
//     // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
//     public static final Transform3d ROBOT_TO_CAM =
//             new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
//     // The layout of the AprilTags on the field
//     public static final AprilTagFieldLayout kTagLayout =
//             AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
//     // The standard deviations of our vision estimated poses, which affect correction rate
//     // (Fake values. Experiment and determine estimation noise on an actual robot.)
//     public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
//     public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
//     private static LocalizerTest m_instance = new LocalizerTest();

package frc.robot.Subsystems.Swerve;                                                                        
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Utils.LocalizationCamera;
import frc.robot.Utils.EverKit.Periodic;
    
public class SwerveLocalizer implements Periodic, SwerveConsts{
    
    private static final LocalizationCamera[] CAMS = { 
        new LocalizationCamera(null, null, null, null, null),
        new LocalizationCamera(null, null, null, null, null)
    }; 

    private static final double FIELD_WIDTH = 0;
    private static final double FIELD_HEIGHT = 0;
    private static final double MAX_CAMERA_HEIGHT = 0;
    private static final double MAX_DISTANCE_FROM_TAG = 0;
    
    private static SwerveLocalizer m_instance = new SwerveLocalizer();
    private ArrayList<LocalizationCamera> m_cams;
    private SwerveDrivePoseEstimator m_poseEstimator;
    private AprilTagFieldLayout m_fieldLayout;

    private SwerveLocalizer(){
        m_cams = new ArrayList<>(Arrays.asList(CAMS));
        m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(modulesPositions[0].x, modulesPositions[0].y),
            new Translation2d(modulesPositions[1].x, modulesPositions[1].y),
            new Translation2d(modulesPositions[2].x, modulesPositions[2].y),
            new Translation2d(modulesPositions[3].x, modulesPositions[3].y)
        );
    
        m_poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            Swerve.getInstance().getGyroRotation(),
            Swerve.getInstance().getModulesPositions(),
            new Pose2d());

        start(PeriodicTime.kRobotPeriodic);
    }
    
    public static SwerveLocalizer getInstance(){
        return m_instance;
    }

    @Override
    public void periodic() {
        // update odometry
        m_poseEstimator.update(Swerve.getInstance().getGyroRotation(), Swerve.getInstance().getModulesPositions());

        // update vision 
        for (LocalizationCamera cam : m_cams) {
            addCameraVisionMeasurements(cam);
        }
    }
    
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
        
        int numTags = 0;
        double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : est.get().targetsUsed) {
                 var tagPose = m_fieldLayout.getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(est.get().estimatedPose.toPose2d().getTranslation());
             }
             avgDist /= numTags;

            boolean isTooFar = avgDist > MAX_DISTANCE_FROM_TAG;
    
            return !outOfField && !aboveCamera && !underGround && !noTags && !isTooFar;
        
    }
    
        private void addCameraVisionMeasurements(LocalizationCamera cam){
            Optional<EstimatedRobotPose> est = cam.getEstimatedGlobalPose();
            
            if(!takeVisionPoseEstimation(est))
                return;
            m_poseEstimator.addVisionMeasurement(est.get().estimatedPose.toPose2d(), est.get().timestampSeconds, cam.getEstimationStdDevs());        
        }
    
        
    }
    
