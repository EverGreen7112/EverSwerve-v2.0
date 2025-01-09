package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.Subsystems.Vision.LocalizationVision;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.EverKit.Periodic.PeriodicTime;
import frc.robot.Utils.Math.SwerveToWpi;
import frc.robot.Utils.Math.Vector2d;

public class SwerveLocalizerTest implements Periodic, SwerveConsts {

    private static SwerveLocalizerTest m_instance = new SwerveLocalizerTest();

    private SwerveDrivePoseEstimator m_poseEstimator;
    
    private SwerveLocalizerTest() {
        
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

    public static SwerveLocalizerTest getInstance() {
        return m_instance;
    }

    @Override
    public void periodic() {
        // update odometry
        m_poseEstimator.update(Swerve.getInstance().getGyroRotation(), Swerve.getInstance().getModulesPositions());
    }

    /**
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

}
