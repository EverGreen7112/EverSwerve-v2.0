
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swerve.SwerveLocalizer;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveAutoController;
import frc.robot.Utils.LocalizationCamera;
import frc.robot.Utils.EverKit.Periodic;

public class Robot extends TimedRobot {

  public static ArrayList<Periodic> robotPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> teleopPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> testPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> autonomousPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> simulationPeriodicFuncs = new ArrayList<Periodic>();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private static Field2d m_field; 


  @Override
  public void robotInit() {
    
    m_robotContainer = new RobotContainer();
    Swerve.getInstance().resetGyro();

    //create and add robot field data to dashboard
    m_field = new Field2d();
    SmartDashboard.putData("field", m_field);
    

    SwerveAutoController.getInstance().addChoosersToDashboard();

   
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    for (Periodic method : robotPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

   
    // update the robot position of dashboard
    m_field.setRobotPose(SwerveLocalizer.getInstance().getCurrentPoint().getX(),
                         SwerveLocalizer.getInstance().getCurrentPoint().getY(),
                        new Rotation2d(Math.toRadians(SwerveLocalizer.getInstance().getFieldOrientedAngle())));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = SwerveAutoController.getInstance().getAutoCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    for (Periodic method : autonomousPeriodicFuncs) {
      try {
        method.periodic();

      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    for(int i = 0; i < 4; i++){
      // Swerve.getInstance().getModules()[i].setState(1, 90);
      // Swerve.getInstance().getModules()[i].setSteer(0.1);
    }

  }

  @Override
  public void teleopPeriodic() { 

    for (Periodic method : teleopPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }

    }    
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    for (Periodic method : testPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void testExit() {}  

  @Override
  public void simulationPeriodic() {
    for (Periodic method : simulationPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }


  
  
}
