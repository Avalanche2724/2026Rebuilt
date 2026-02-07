package frc.robot.util;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.HashMap;
import java.util.Map;

public final class Utils {
  private static final int NUM_CONFIG_APPLY_ATTEMPTS = 2;

  private static final Map<String, Alert> configApplyAlerts = new HashMap<>();

  private Utils() {}

  public static boolean applyTalonFxConfigWithRetry(
      String name, TalonFX motor, TalonFXConfiguration configs) {
    StatusCode status = motor.getConfigurator().apply(configs);
    for (int i = 1; i < NUM_CONFIG_APPLY_ATTEMPTS && !status.isOK(); i++) {
      status = motor.getConfigurator().apply(configs);
    }

    if (status.isOK()) {
      var alert = configApplyAlerts.get(name);
      if (alert != null) {
        alert.set(false);
      }
      return true;
    }

    var alert =
        configApplyAlerts.computeIfAbsent(
            name,
            _name ->
                new Alert(
                    "Phoenix Config",
                    "Failed to apply TalonFX config: " + name,
                    Alert.AlertType.kError));
    alert.setText(
        "Failed to apply TalonFX config (" + name + ", id=" + motor.getDeviceID() + "): " + status);
    alert.set(true);
    return false;
  }

  /**
   * Updates a {@link DCMotorSim} and applies its rotor position/velocity to one or more TalonFX sim
   * states (e.g. a master + follower).
   *
   * <p>Input voltage is taken from the first provided sim state.
   */
  public static void updateTalonFxRotorSim(
      double dtSeconds, DCMotorSim motorSim, TalonFXSimState... simStates) {
    if (simStates.length == 0) {
      throw new IllegalArgumentException("Expected at least one TalonFXSimState");
    }

    double batteryVoltage = RobotController.getBatteryVoltage();
    for (var simState : simStates) {
      simState.setSupplyVoltage(batteryVoltage);
    }

    motorSim.setInputVoltage(simStates[0].getMotorVoltage());
    motorSim.update(dtSeconds);

    double rotorPositionRot = motorSim.getAngularPositionRotations();
    double rotorVelocityRps =
        RotationsPerSecond.convertFrom(motorSim.getAngularVelocityRadPerSec(), RadiansPerSecond);

    for (var simState : simStates) {
      simState.setRawRotorPosition(rotorPositionRot);
      simState.setRotorVelocity(rotorVelocityRps);
    }
  }
}
