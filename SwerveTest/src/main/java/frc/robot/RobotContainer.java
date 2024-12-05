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
import frc.robot.Commands.Climb.ExtendLeft;
import frc.robot.Commands.Climb.ExtendRight;
import frc.robot.Commands.Climb.RetractLeft;
import frc.robot.Commands.Climb.RetractRight;
import frc.robot.Commands.Intake.EmitNote;
import frc.robot.Commands.Intake.IntakeNote;
import frc.robot.Commands.Shooter.ShootToAmp;
import frc.robot.Commands.Shooter.AutomaticShootToSpeaker;
import frc.robot.Commands.Shooter.ReleaseNote;
import frc.robot.Commands.Shooter.ShootFromSideOfAmp;
import frc.robot.Commands.Shooter.TurnShooterTo;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Utils.Math.SwerveToWpi;

public class RobotContainer {

  private static final int CHASSIS_PORT = 0;
  private static final int OPERATOR_PORT = 1;

   //controllers
  public static final CommandXboxController chassis = new CommandXboxController(CHASSIS_PORT);
  public static final CommandXboxController operator = new CommandXboxController(OPERATOR_PORT);
  
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
              return  Robot.getAlliance() == DriverStation.Alliance.Red;
            },
            Swerve.getInstance() // Reference to this subsystem to set requirements
    );
    
  }



  private void configureBindings() {
    NamedCommands.registerCommand("start intake", new InstantCommand(()->{Intake.getInstance().intakeNote();
                                                                               Shooter.getInstance().containNote();
                                                                               Shooter.getInstance().turnToIntake();} ));
    
    NamedCommands.registerCommand("stop intake", new InstantCommand(()->{
                                                                              Intake.getInstance().stop();
                                                                              Shooter.getInstance().stopIntake();}));
    
                                                                              
    NamedCommands.registerCommand("start shoot from side of amp", new InstantCommand(() -> {Shooter.getInstance().turnTo(113);}).andThen(new ShootFromSideOfAmp().withTimeout(2)));
    NamedCommands.registerCommand("release note", new ReleaseNote().withTimeout(0.5));
  
    //intake
    operator.a().whileTrue(new IntakeNote());
    operator.b().whileTrue(new ReleaseNote());
    operator.start().whileTrue(new EmitNote());
    operator.x().whileTrue(new ShootToAmp());
    operator.y().whileTrue(new AutomaticShootToSpeaker());

    //climber
    operator.rightBumper().whileTrue(new ExtendRight());//RB
    operator.leftBumper().whileTrue(new ExtendLeft());//LB
    operator.rightTrigger().whileTrue(new RetractRight());//RT
    operator.leftTrigger().whileTrue(new RetractLeft());//RL
    

  }

  
}
