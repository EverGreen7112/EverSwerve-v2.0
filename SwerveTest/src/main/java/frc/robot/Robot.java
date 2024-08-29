// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.lang.reflect.Method;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.SwerveConsts;
import frc.robot.Utils.EverKit.Periodic;
import frc.robot.Utils.EverKit.Implementations.Encoders.EverSparkInternalEncoder;
import frc.robot.Utils.Math.Vector2d;

public class Robot extends TimedRobot {
  public static ArrayList<Periodic> robotPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> teleopPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> testPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> autonomousPeriodicFuncs = new ArrayList<Periodic>();
  public static ArrayList<Periodic> simulationPeriodicFuncs = new ArrayList<Periodic>();

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    Swerve.getInstance();
    m_robotContainer = new RobotContainer();
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
    
    // SmartDashboard.putNumber("POS tl", SwerveConsts.SWERVE_MODULES[0].getAbsPos());
    // SmartDashboard.putNumber("POS tr", SwerveConsts.SWERVE_MODULES[1].getAbsPos());
    // SmartDashboard.putNumber("POS dl", SwerveConsts.SWERVE_MODULES[2].getAbsPos());
    // SmartDashboard.putNumber("POS dr", SwerveConsts.SWERVE_MODULES[3].getAbsPos());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
  }


  CommandXboxController controller = new CommandXboxController(0);

  @Override
  public void teleopPeriodic() {
    for (Periodic method : teleopPeriodicFuncs) {
      try {
        method.periodic();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    Vector2d vec = new Vector2d(controller.getLeftX(), controller.getLeftY() * -1);
    
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
