package frc.robot.Utils.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Subsystems.Swerve.SwervePoint;

/**     
 * class including util function for converting to WPIlib's classes
 * all of these function are taken from pathplanner's documentation for
 * configuring an auto
 */
public class SwerveToWpi {
    private static Swerve m_swerve = Swerve.getInstance();

    public static Pose2d getPos() {
        SwervePoint currentPoint = SwerveLocalizer.getInstance().getCurrentPoint();
        return new Pose2d(currentPoint.getX(),
                currentPoint.getY(),
                new Rotation2d(Math.toRadians(currentPoint.getAngle())));
    }

    public static void resetPos(Pose2d pos) {
        SwerveLocalizer.getInstance().setCurrentPoint(new SwervePoint(pos.getX(),
                pos.getY(),
                pos.getRotation().getDegrees()));
    }

    public static ChassisSpeeds getRobotRelativeSpeeds() {
        ChassisSpeeds speeds = new ChassisSpeeds();
        speeds.omegaRadiansPerSecond = Math.toRadians(m_swerve.getAngularVelocity());
        speeds.vxMetersPerSecond = m_swerve.getRobotOrientedVelocity().x;
        speeds.vyMetersPerSecond = m_swerve.getRobotOrientedVelocity().y;
        return speeds;
    }

    public static void driveRobotRelative(ChassisSpeeds speeds) {
        m_swerve.drive(new Vector2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), false,
                Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

}
