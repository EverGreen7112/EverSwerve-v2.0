package frc.robot.Commands.Swerve;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;

public class DriveToBranch extends Command {

    private ReefFace m_reefFace;
    private boolean m_isRightBranch;

    public DriveToBranch(ReefFace reefFace, boolean isRightBranch) {
        m_reefFace = reefFace;
        m_isRightBranch = isRightBranch;
    }

    @Override
    public void initialize() {
        Pose2d targetBranch = (m_isRightBranch) ? m_reefFace.getRightBranchPose() : m_reefFace.getLeftBranchPose();
        SmartDashboard.putString("target", targetBranch.toString());

        SwerveAutoController.getInstance().generateDriveToCommand(
                new GoalEndState(0, m_reefFace.getFacePose().getRotation()),
                targetBranch.plus(new Transform2d(-0.2, 0, new Rotation2d())),
                targetBranch).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
