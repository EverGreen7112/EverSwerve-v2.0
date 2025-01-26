package frc.robot.Subsystems.Swerve;

import java.util.Currency;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Utils.Math.Funcs;

public class SwerveAutoController {

    private static final PIDConstants TRANSLATION_PID =  new PIDConstants(5.0, 0.0, 0.0),
                                      ROTATION_PID = new PIDConstants(5.0, 0.0 ,0.0);
    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(1.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    private static SwerveAutoController m_instance = new SwerveAutoController();
    private SendableChooser<Command> m_autoChooser;
    private SendableChooser<Alliance> m_allianceChooser;

    private SwerveAutoController(){

        RobotConfig config = null;

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            SmartDashboard.putBoolean("couldnt load robot config expect problems in auto", false);
        }

        AutoBuilder.configure(
            SwerveLocalizer.getInstance()::getCurrentPoint, // Robot pose supplier
            SwerveLocalizer.getInstance()::setCurrentPoint, // Method to reset odometry
            Swerve.getInstance()::getRobotOrientedSpeeds, // ChassisSpeeds supplier
            ((speeds, feedforwards) -> Swerve.getInstance().driveRobotOrientedBySpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( 
                TRANSLATION_PID, // Translation PID constants
                ROTATION_PID // Rotation PID constants
            ),
            config, 
            () -> { //flip path
              return  getAlliance() == DriverStation.Alliance.Red;
            },
            Swerve.getInstance()
        );

        m_autoChooser = new SendableChooser<Command>();
        m_autoChooser.addOption("test", new PathPlannerAuto("test"));

        m_allianceChooser = new SendableChooser<Alliance>();
        m_allianceChooser.addOption("blue", Alliance.Blue);
        m_allianceChooser.addOption("red", Alliance.Red);
    }

    public static SwerveAutoController getInstance(){
        return m_instance;
    }

    public void addChoosersToDashboard(){
        SmartDashboard.putData("auto", m_autoChooser);
        SmartDashboard.putData("alliance", m_allianceChooser);
    }

    public Command getAutoCommand(){
        return m_autoChooser.getSelected();
    }

    public Alliance getAlliance(){
        return m_allianceChooser.getSelected();
    }

    public Command generateDriveToCommand(GoalEndState endState, Pose2d...waypoints){
        PathPlannerPath path = new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(waypoints),
            PATH_CONSTRAINTS,
            null,
            endState);
        path.preventFlipping = true;
        return AutoBuilder.pathfindThenFollowPath(path, PATH_CONSTRAINTS);
    }
    
}
