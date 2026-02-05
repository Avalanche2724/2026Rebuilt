// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class RobotContainer {
  public Shooter shooter = new Shooter();
  public Spindexer spindexer = new Spindexer();

  public RobotContainer() {
    SmartDashboard.putNumber("SHOOTER SPEED", 33.5);
    SmartDashboard.putNumber("INDEXER SPEED", 2.5);

    configureBindings();
  }

  private CommandXboxController driver = new CommandXboxController(0);

  private void configureBindings() {
    driver
        .a()
        .whileTrue(spindexer.runAtVolts(() -> SmartDashboard.getNumber("INDEXER SPEED", 2.5)));
    driver.b().whileTrue(shooter.runAtSpeed(() -> SmartDashboard.getNumber("SHOOTER SPEED", 33.5)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
