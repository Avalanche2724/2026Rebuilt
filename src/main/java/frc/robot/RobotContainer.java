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
  private static final String SHOOTER_SPEED_DASHBOARD_KEY = "SHOOTER SPEED";
  private static final double SHOOTER_SPEED_DEFAULT_RPS = 33.5;

  private static final String INDEXER_SPEED_DASHBOARD_KEY = "INDEXER SPEED";
  private static final double INDEXER_SPEED_DEFAULT_VOLTS = 2.5;

  private static final int DRIVER_CONTROLLER_PORT = 0;
  private static final double SIM_THREAD_PERIOD_SECONDS = 0.005;

  public final Shooter shooter = new Shooter();
  public final Spindexer spindexer = new Spindexer();
  private final CommandXboxController driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);

  private double lastSimTimeSeconds;
  private Notifier simNotifier;

  public RobotContainer() {
    SmartDashboard.putNumber(SHOOTER_SPEED_DASHBOARD_KEY, SHOOTER_SPEED_DEFAULT_RPS);
    SmartDashboard.putNumber(INDEXER_SPEED_DASHBOARD_KEY, INDEXER_SPEED_DEFAULT_VOLTS);

    if (RobotBase.isSimulation()) {
      startSimThread();
    }

    configureBindings();
  }

  private void startSimThread() {
    lastSimTimeSeconds = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTimeSeconds;
              lastSimTimeSeconds = currentTime;

              shooter.updateSim(deltaTime);
              spindexer.updateSim(deltaTime);
            });
    simNotifier.startPeriodic(SIM_THREAD_PERIOD_SECONDS);
  }

  private void configureBindings() {
    driver
        .a()
        .whileTrue(
            spindexer.runAtVolts(
                () ->
                    SmartDashboard.getNumber(
                        INDEXER_SPEED_DASHBOARD_KEY, INDEXER_SPEED_DEFAULT_VOLTS)));
    driver
        .b()
        .whileTrue(
            shooter.runAtSpeed(
                () ->
                    SmartDashboard.getNumber(
                        SHOOTER_SPEED_DASHBOARD_KEY, SHOOTER_SPEED_DEFAULT_RPS)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
