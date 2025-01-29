package frc.robot.Commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;

public class DriveToClosestBranch extends Command {

    private boolean m_isRightBranch;

    public DriveToClosestBranch(boolean isRightBranch) {
        m_isRightBranch = isRightBranch;
    }

    @Override
    public void initialize() {

        ReefFace[] reef = (SwerveAutoController.getInstance().getAlliance() == Alliance.Blue ? ReefFace.BLUE_REEF : ReefFace.RED_REEF);
        SmartDashboard.putBoolean("isBlue", SwerveAutoController.getInstance().getAlliance() == Alliance.Blue);
        Pose2d currentPoint = SwerveLocalizer.getInstance().getCurrentPoint();
        double minDis = getDis(currentPoint, reef[0].getFacePose());
        ReefFace closestFace = reef[0];
        for (int i = 1; i < 6; i++) {
            double currentDistance = getDis(currentPoint, reef[i].getFacePose());
            if (minDis > currentDistance) {
                minDis = currentDistance;
                closestFace = reef[i];
            }
        }
        SmartDashboard.putString("closestFace", closestFace.toString());
        (new DriveToBranch(closestFace, m_isRightBranch)).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    private double getDis(Pose2d first, Pose2d second){
        return (first.minus(second)).getTranslation().getNorm();
    }

}
