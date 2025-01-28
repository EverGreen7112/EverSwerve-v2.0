package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ReefFace {
    private Pose2d m_facePose;
    private Pose2d m_leftBranchPose;
    private Pose2d m_rightBranchPose;

    public static final ReefFace[] BLUE_REEF = {
        new ReefFace(3.66, 4.03, 0  , 3.66,4.2,0,3.66,3.83,0),
        new ReefFace(4.07, 3.31, 60 , 3.93,3.395,60,4.21,3.225,60),
        new ReefFace(4.90, 3.31, 120, 4.76,3.225,120,5.04,3.395,120),
        new ReefFace(5.32, 4.03, 180, 5.32,3.83,180,5.32,4.2,180),
        new ReefFace(4.90, 4.75, 240, 5.04,4.665,240,4.76,4.835,240),
        new ReefFace(4.07, 4.75, 300, 4.21,4.835,300,3.93,4.665,300)
    };

    public static final ReefFace[] RED_REEF = {
        new ReefFace(12.23, 4.03, 0,    12.23,4.2,0,12.23,3.83,0),
        new ReefFace(12.64, 3.31, 60,   12.5,3.395,60,12.78,3.225,60),
        new ReefFace(13.47, 3.31, 120,  13.33,3.225,120,13.61,3.395,120),
        new ReefFace(13.89, 4.03, 180,  13.89,3.83,180,13.89,4.2,180),
        new ReefFace(13.47, 4.75, 240,  13.61,4.665,240,13.33,4.835,240),
        new ReefFace(12.64, 4.75, 300,  12.78,4.835,0,12.5,4.665,0)

    };

    public ReefFace(Pose2d facePose, Pose2d leftBranchPose, Pose2d rightBranchPose){
        m_facePose = facePose;
        m_leftBranchPose = leftBranchPose;
        m_rightBranchPose = rightBranchPose;
    }

    public ReefFace(double x1, double y1, double ang1, double x2, double y2, double ang2, double x3, double y3, double ang3){
        this(new Pose2d(x1, y1, new Rotation2d(Math.toRadians(ang1))), 
             new Pose2d(x2, y2, new Rotation2d(Math.toRadians(ang2))),
             new Pose2d(x3, y3, new Rotation2d(Math.toRadians(ang3))));
    }

    

    public Pose2d getFacePose(){
        return m_facePose;
    }

    public Pose2d getLeftBranchPose(){
        return m_leftBranchPose;
    }

    public Pose2d getRightBranchPose(){
        return m_rightBranchPose;
    }
    
}
