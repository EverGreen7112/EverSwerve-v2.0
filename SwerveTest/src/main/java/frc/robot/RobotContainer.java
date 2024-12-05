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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Climb.ExtendLeftClimber;
import frc.robot.Commands.Climb.ExtendRightClimber;
import frc.robot.Commands.Climb.RetractLeftClimber;
import frc.robot.Commands.Climb.RetractRightClimber;
import frc.robot.Commands.Intake.EmitNote;
import frc.robot.Commands.Intake.IntakeNote;
import frc.robot.Commands.Shooter.ShootToAmp;
import frc.robot.Commands.Shooter.ShootToSpeaker;
import frc.robot.Commands.Shooter.AlternateAutoShootToSpeaker;
import frc.robot.Commands.Shooter.AutomaticShootToSpeaker;
import frc.robot.Commands.Shooter.CalibrateShooter;
import frc.robot.Commands.Shooter.ReleaseNote;
import frc.robot.Commands.Shooter.ShootFromNotSideOfAmp;
import frc.robot.Commands.Shooter.ShootFromSideOfAmp;
import frc.robot.Commands.Shooter.TurnShooterTo;
import frc.robot.Commands.Swerve.DriveByJoysticks;
import frc.robot.Commands.Swerve.DriveByJoysticks.SpeedMode;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConsts;
import frc.robot.Subsystems.Swerve.SwitchSpeedMode;
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

  public static final Command shootToAmpCommand = new ShootToAmp();
  public static final Command shootToSpeakerCommand = new ShootToSpeaker();
  public static final Command intakeCommand = new IntakeNote();
  public static final Command releaseCommand = new ReleaseNote();
  public static final Command emitNoteCommand = new EmitNote();
  public static final Command extendRightClimberCommand = new ExtendRightClimber();
  public static final Command extendLeftClimberCommand = new ExtendLeftClimber();
  public static final Command retractRightClimberCommand = new RetractRightClimber();
  public static final Command retractLeftClimberCommand = new RetractLeftClimber();
  public static final Command automaticSpeaker = new AutomaticShootToSpeaker();


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
    // //autonomous 
    NamedCommands.registerCommand("start intake", new InstantCommand(()->{Intake.getInstance().intakeNote();
                                                                               Shooter.getInstance().intakeNote();
                                                                               Shooter.getInstance().turnToIntake();} ));
    
    NamedCommands.registerCommand("stop intake", new InstantCommand(()->{
                                                                              Intake.getInstance().stop();
                                                                              Shooter.getInstance().stopIntake();}));
    
                                                                              
    NamedCommands.registerCommand("shoot from side of amp", new ShootFromSideOfAmp().withTimeout(2));
    NamedCommands.registerCommand("shoot from not side of amp", new ShootFromNotSideOfAmp().withTimeout(2));
    NamedCommands.registerCommand("release note", new ReleaseNote().withTimeout(0.4));
    NamedCommands.registerCommand("shoot from middle", new ShootToSpeaker().withTimeout(1.5)); 
  }

  private void configureBindings() {

    //chassis
    DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> chassisStart.getAsBoolean(), () -> chassisA.getAsBoolean());
    
    chassisBack.onTrue(new InstantCommand(() -> {Swerve.getInstance().resetGyro();}));
    chassisRT.whileTrue(new SwitchSpeedMode(SpeedMode.kTurbo));
    chassisLT.whileTrue(new SwitchSpeedMode(SpeedMode.kSlow));
     Swerve.getInstance().setDefaultCommand(teleop);
    
    //operator
    operatorY.whileTrue(automaticSpeaker);
    operatorA.whileTrue(intakeCommand);
    operatorB.whileTrue(releaseCommand);
    operatorStart.whileTrue(emitNoteCommand);
    operatorX.whileTrue(shootToAmpCommand);
    operatorPovUp.whileTrue(shootToSpeakerCommand);
    operatorRB.whileTrue(extendRightClimberCommand);//RB
    operatorLB.whileTrue(extendLeftClimberCommand);//LB
    operatorRT.whileTrue(retractRightClimberCommand);//RT
    operatorLT.whileTrue(retractLeftClimberCommand);//LT
    operatorPovRight.whileTrue(new AlternateAutoShootToSpeaker());


  }

  
}
