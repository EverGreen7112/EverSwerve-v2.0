// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveByJoysticks;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Utils.Math.SwerveToWpi;

public class RobotContainer {

  public static final CommandXboxController chassis = new CommandXboxController(0);
  
  //command instances
  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX());

  public RobotContainer() {
    configureBindings();
    
    ReplanningConfig a = new ReplanningConfig(true, false);
    

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            SwerveToWpi::getPos, // Robot pose supplier
            SwerveToWpi::resetPos, // Method to reset odometry (will be called if your auto has a starting pose)
            SwerveToWpi::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            SwerveToWpi::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(
                    new PIDConstants(2.5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.5, 0.0, 0.0), // Rotation PID constants
                    2,
                    SwerveConsts.ROBOT_RADIUS,
                    a
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              // var alliance = DriverStation.getAlliance();
              // if (alliance.isPresent()) {
              //   return alliance.get() == DriverStation.Alliance.Red;
              // }
              
              // return false;
              return false;
            },
            Swerve.getInstance() // Reference to this subsystem to set requirements
    );
    
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test");
  }
}
