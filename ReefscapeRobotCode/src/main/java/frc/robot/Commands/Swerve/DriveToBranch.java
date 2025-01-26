package frc.robot.Commands.Swerve;

import java.util.Vector;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.ReefFace;
import frc.robot.Utils.Math.Funcs;
import frc.robot.Utils.Math.Vector2d;

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

        SwerveAutoController.getInstance().generateDriveToCommand(
                new GoalEndState(0, m_reefFace.getFacePose().getRotation()),
                SwerveLocalizer.getInstance().getCurrentPoint(),
                targetBranch).schedule();

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
