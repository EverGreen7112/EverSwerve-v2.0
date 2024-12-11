// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Intake.EmitNote;
import frc.robot.Commands.Intake.IntakeNote;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Commands.Swerve.DriveByJoysticks.SpeedMode;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Utils.Math.SwerveToWpi;

public class RobotContainer {

  private static final int CHASSIS_PORT = 0;
  private static final int OPERATOR_PORT = 1;

   //controllers
  public static final CommandXboxController chassis = new CommandXboxController(CHASSIS_PORT);
  public static final CommandXboxController operator = new CommandXboxController(OPERATOR_PORT);

  //Triggers
  public static final Trigger operatorA = operator.a();
  public static final Trigger operatorB = operator.b();
  public static final Trigger operatorX = operator.x();
  public static final Trigger operatorY = operator.y();
  public static final Trigger operatorPovUp = operator.povUp();
  public static final Trigger operatorPovRight = operator.povRight();
  public static final Trigger operatorRB = operator.rightBumper();
  public static final Trigger operatorLB = operator.leftBumper();
  public static final Trigger operatorRT = operator.rightTrigger();
  public static final Trigger operatorLT = operator.leftTrigger();
  public static final Trigger operatorStart = operator.start();

  public static final Trigger chassisStart = chassis.start();
  public static final Trigger chassisBack = chassis.back();
  public static final Trigger chassisA = chassis.a();
  public static final Trigger chassisRT = chassis.rightTrigger();
  public static final Trigger chassisLT = chassis.leftTrigger();


  public RobotContainer() {
    registerNamedCommands();
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

  private void registerNamedCommands(){
  
  }

  private void configureBindings() {

    //chassis
    DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> chassisStart.getAsBoolean(), () -> chassisA.getAsBoolean());
    
    chassisBack.onTrue(new InstantCommand(() -> {Swerve.getInstance().resetGyro();}));
    
    Swerve.getInstance().setDefaultCommand(teleop);
    

  }

  
}
