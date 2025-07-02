// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

public class RobotContainer {
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final Drive drive = new Drive();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverXbox.a().onTrue(drive.printFrontLeft());
    driverXbox.b().onTrue(drive.printFrontRight());
    driverXbox.x().onTrue(drive.printBackLeft());
    driverXbox.y().onTrue(drive.printBackRight());
    drive.setDefaultCommand(
        drive.simpleDrive(() -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX(), () -> driverXbox.getRightX()));
    // drive.setDefaultCommand(drive.test());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
  }
}
