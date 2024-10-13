// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Intake.EjectNote;
import frc.robot.Commands.Intake.IntakeNote;
import frc.robot.Commands.Shooter.ChargeNote;
import frc.robot.Commands.Shooter.ReleaseNote;
import frc.robot.Commands.Shooter.RotateTo;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterConsts;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Utils.Math.SwerveToWpi;

public class RobotContainer {

  public static final CommandXboxController chassis = new CommandXboxController(0);
  
  //command instances
  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> chassis.back().getAsBoolean());

  public RobotContainer() {
    configureBindings();

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
                    new ReplanningConfig(true, false)
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

  private void configureBindings() {
    NamedCommands.registerCommand("start intake", new InstantCommand(()->{Intake.getInstance().intake();
                                                                               Shooter.getInstance().pullNote(-0.5);
                                                                               Shooter.getInstance().turnToAngle(ShooterConsts.AIM_MOTOR_MIN_ANGLE);}));
    
    NamedCommands.registerCommand("stop intake", new InstantCommand(()->{
                                                                              Shooter.getInstance().pullNote(0);}));
    

    //buttons
    chassis.a().whileTrue(new IntakeNote(0.5));
    chassis.start().whileTrue(new EjectNote(0.5));
    chassis.x().whileTrue(new ChargeNote());
    chassis.y().whileTrue(new ReleaseNote());
    chassis.b().onTrue(new RotateTo(60));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("3 note auto - middle");
  }
}
