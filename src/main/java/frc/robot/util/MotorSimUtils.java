package frc.robot.util;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class MotorSimUtils {
  private MotorSimUtils() {}

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
