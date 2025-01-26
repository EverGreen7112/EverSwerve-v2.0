package frc.robot.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class LocalizationCamera {
    private final PhotonCamera m_cam;
    private final PhotonPoseEstimator m_poseEstimator;
    private Matrix<N3, N1> m_singleTagStdDevs;
    private Matrix<N3, N1> m_multiTagStdDevs;
    private Matrix<N3, N1> m_curStdDevs;
 
     public LocalizationCamera(String camName, AprilTagFieldLayout tagFieldLayout, Transform3d robotToCam, Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevs) {
         m_cam = new PhotonCamera(camName);
         m_poseEstimator =
                 new PhotonPoseEstimator(tagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
         m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
         m_singleTagStdDevs = singleTagStdDevs;
         m_multiTagStdDevs = multiTagStdDevs;
         
     }
 
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         List<PhotonPipelineResult> res =  m_cam.getAllUnreadResults();
         for (PhotonPipelineResult change : res) {
             visionEst = m_poseEstimator.update(change);
             updateEstimationStdDevs(visionEst, change.getTargets());
         }  

         return visionEst;
     }
 
    
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            m_curStdDevs = m_singleTagStdDevs;
 
        } else {
             // Pose present. Start running Heuristic
             var estStdDevs = m_singleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = m_poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 m_curStdDevs = m_singleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = m_multiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 m_curStdDevs = estStdDevs;
             }
         }
     }
 
     
     public Matrix<N3, N1> getEstimationStdDevs() {
         return m_curStdDevs;
     }
 
    
 }