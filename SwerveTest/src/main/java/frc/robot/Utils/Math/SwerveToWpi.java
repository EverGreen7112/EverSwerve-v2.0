package frc.robot.Utils.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Swerve.Swerve;

/**
 * class including util function for converting to WPIlib's classes
 */
public class SwerveToWpi {
    private static Swerve m_swerve = Swerve.getInstance();

    public ChassisSpeeds getChassisSpeeds(){
        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.omegaRadiansPerSecond = Math.toRadians(m_swerve.getAngularVelocity());
        speeds.vxMetersPerSecond = m_swerve.getVelocity().x;
        speeds.vyMetersPerSecond = m_swerve.getVelocity().y;
        return speeds;
    }

    public Pose2d getPos(){
        return null;
    }



}
