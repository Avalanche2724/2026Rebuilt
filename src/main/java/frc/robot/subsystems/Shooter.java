package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private static final int MAIN_MOTOR_ID = 61;
  private static final int FOLLOW_MOTOR_ID = 62;

  private final TalonFX mainMotor = new TalonFX(MAIN_MOTOR_ID);
  private final TalonFX followMotor = new TalonFX(FOLLOW_MOTOR_ID);

  // Simulation: simple rotor model driven by the TalonFX's simulated motor voltage.
  private final TalonFXSimState mainSim = mainMotor.getSimState();
  private final TalonFXSimState followSim = followMotor.getSimState();
  private final DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(2),
              0.002, // moment of inertia (kg*m^2), tune as needed
              1.0 // output-to-input (rotor) ratio
              ),
          DCMotor.getKrakenX60Foc(2));

  public Shooter() {
    var configs = new TalonFXConfiguration();
    configs.CurrentLimits.StatorCurrentLimit = 200.0;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 120.0;
    configs.CurrentLimits.SupplyCurrentLowerTime = 2.0;
    configs.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.Slot0.kP = 0.5;
    configs.Slot0.kS = 0.28;
    configs.Slot0.kV = 0.124;

    followMotor.setControl(new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    mainMotor.getConfigurator().apply(configs);
    followMotor.getConfigurator().apply(configs);

    mainMotor.getMotorVoltage().setUpdateFrequency(250.0);
    mainMotor.getTorqueCurrent().setUpdateFrequency(250.0);
    mainMotor.getVelocity().setUpdateFrequency(250.0);
    mainMotor.getSupplyCurrent().setUpdateFrequency(250.0);
    mainMotor.getSupplyVoltage().setUpdateFrequency(250.0);
  }

  public Command runAtSpeed(DoubleSupplier speedRps) {
    return this.startEnd(
        () -> {
          mainMotor.setControl(new VelocityVoltage(speedRps.getAsDouble()));
        },
        () -> {
          mainMotor.setControl(new VoltageOut(0.0));
        });
  }

  // CTRE Phoenix 6 simulation requires periodically updating rotor position/velocity.
  public void updateSim(double dtSeconds) {
    double batteryVoltage = RobotController.getBatteryVoltage();
    mainSim.setSupplyVoltage(batteryVoltage);
    followSim.setSupplyVoltage(batteryVoltage);

    shooterSim.setInputVoltage(mainSim.getMotorVoltage());
    shooterSim.update(dtSeconds);

    double rotorPositionRot = shooterSim.getAngularPositionRotations();
    double rotorVelocityRps =
        RotationsPerSecond.convertFrom(shooterSim.getAngularVelocityRadPerSec(), RadiansPerSecond);

    mainSim.setRawRotorPosition(rotorPositionRot);
    mainSim.setRotorVelocity(rotorVelocityRps);

    followSim.setRawRotorPosition(rotorPositionRot);
    followSim.setRotorVelocity(rotorVelocityRps);
  }
}
