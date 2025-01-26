package frc.robot.Commands.Swerve;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Math.Funcs;

public class DriveToPose extends Command{
    
    @Override
    public void initialize() {
        SwerveAutoController.getInstance().generateDriveToCommand(
            new GoalEndState(0, Funcs.degreesToRotation2d(180)),
            new Pose2d(13.9, 5.548, Funcs.degreesToRotation2d(180.0)),
            new Pose2d(14.0, 5.548, Funcs.degreesToRotation2d(180.0))
        ).schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
