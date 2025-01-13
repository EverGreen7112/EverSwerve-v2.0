package frc.robot.Subsystems.LocalizationTest;

import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

public class LocalizationCamera {
    private PhotonCamera m_cam;
    private PhotonPoseEstimator m_poseEstimator;
    private Consumer<Optional<EstimatedRobotPose>> m_onRecieveCall;
    private Optional<EstimatedRobotPose> m_latestResult;

    public LocalizationCamera(String camName, AprilTagFieldLayout fieldLayout, PoseStrategy poseStrategy, Transform3d robotToCam, Consumer<Optional<EstimatedRobotPose>> onRecieveCall){
        m_cam = new PhotonCamera(camName);
        m_poseEstimator = new PhotonPoseEstimator(fieldLayout, poseStrategy, robotToCam);
        m_latestResult = m_poseEstimator.update(m_cam.getLatestResult());
        m_onRecieveCall = onRecieveCall;
    }

    public Optional<EstimatedRobotPose> updateEstimation(){
        
        Optional<EstimatedRobotPose> estPose = m_poseEstimator.update(m_cam.getLatestResult());
        if(estPose != m_latestResult){
            m_onRecieveCall.accept(estPose);
            m_latestResult = estPose;
            return estPose;
        }
        return null;
    }

}
