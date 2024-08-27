// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveByJoysticks;

public class RobotContainer {


  public static final CommandXboxController chassis = new CommandXboxController(0);
  
  //command instances
  public static DriveByJoysticks teleop = new DriveByJoysticks(() -> chassis.getLeftX(), () -> chassis.getLeftY(),
      () -> chassis.getRightX(), () -> true, true);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
