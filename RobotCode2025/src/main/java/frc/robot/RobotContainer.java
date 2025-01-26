// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Intake.EmitNote;
import frc.robot.Commands.Intake.IntakeNote;
import frc.robot.Commands.Swerve.ChangeTeleopSpeedModeCommand;
import frc.robot.Commands.Swerve.DriveToPose;
import frc.robot.Commands.Swerve.LockSwerveAngleCommand;
import frc.robot.Commands.Swerve.RotateByCommand;
import frc.robot.Commands.Swerve.RotateToCommand;
import frc.robot.Commands.Swerve.TeleopDriveCommand;
import frc.robot.Commands.Swerve.ChangeTeleopSpeedModeCommand.SpeedMode;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Utils.Math.Funcs;

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
  public static final Trigger chassisB = chassis.b();
  public static final Trigger chassisRT = chassis.rightTrigger();
  public static final Trigger chassisLT = chassis.leftTrigger();

  public static final TeleopDriveCommand teleopCommand = new TeleopDriveCommand(chassis::getLeftX, chassis::getLeftY, chassis::getRightX);

  public RobotContainer() {
    registerNamedCommands();
    configureBindings();
  }

  private void registerNamedCommands(){
  
  }

  private void configureBindings() {

    //chassis
    Swerve.getInstance().setDefaultCommand(teleopCommand);
    chassisA.onTrue(new RotateByCommand(90));
    chassisB.onTrue(new InstantCommand(()->{Swerve.getInstance().resetGyro();}));
    chassisRT.whileTrue(new ChangeTeleopSpeedModeCommand(SpeedMode.kTurbo));
    chassisLT.whileTrue(new ChangeTeleopSpeedModeCommand(SpeedMode.kSlow));
    chassisBack.onTrue(new InstantCommand(() -> {SwerveLocalizer.getInstance().setCurrentPoint(new Pose2d());}));
    chassisStart.onTrue(new DriveToPose());
    


  }

  
}
