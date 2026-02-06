// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;

public class RobotContainer {
  public Shooter shooter = new Shooter();
  public Spindexer spindexer = new Spindexer();

  private double m_lastSimTime;
  private Notifier m_simNotifier;

  public RobotContainer() {
    SmartDashboard.putNumber("SHOOTER SPEED", 33.5);
    SmartDashboard.putNumber("INDEXER SPEED", 2.5);

    if (RobotBase.isSimulation()) {
      startSimThread();
    }

    configureBindings();
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              final double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              shooter.updateSim(deltaTime);
              spindexer.updateSim(deltaTime);
            });
    m_simNotifier.startPeriodic(0.005);
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
