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
  TalonFX mainMotor = new TalonFX(61);
  TalonFX followMotor = new TalonFX(62);

  // Simulation: simple rotor model driven by the TalonFX's simulated motor voltage.
  private final TalonFXSimState mainSim = mainMotor.getSimState();
  private final TalonFXSimState followSim = followMotor.getSimState();
  private final DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(2),
              0.002, // kShooterJKgM2: moment of inertia, tune as needed
              1.0 // kShooterGearing: output-to-input (rotor) ratio
              ),
          DCMotor.getKrakenX60Foc(2));

  public Shooter() {
    var configs = new TalonFXConfiguration();
    configs.CurrentLimits.StatorCurrentLimit = 200;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 120;
    configs.CurrentLimits.SupplyCurrentLowerTime = 2;
    configs.CurrentLimits.SupplyCurrentLowerLimit = 40;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.Slot0.kP = 0.5;
    configs.Slot0.kS = 0.28;
    configs.Slot0.kV = 0.124;

    followMotor.setControl(new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    mainMotor.getConfigurator().apply(configs);
    followMotor.getConfigurator().apply(configs);

    mainMotor.getMotorVoltage().setUpdateFrequency(250);
    mainMotor.getTorqueCurrent().setUpdateFrequency(250);
    mainMotor.getVelocity().setUpdateFrequency(250);
    mainMotor.getSupplyCurrent().setUpdateFrequency(250);
    mainMotor.getSupplyVoltage().setUpdateFrequency(250);
  }

  public Command runAtSpeed(DoubleSupplier speed) {
    return this.startEnd(
        () -> {
          mainMotor.setControl(new VelocityVoltage(speed.getAsDouble()));
        },
        () -> {
          mainMotor.setControl(new VoltageOut(0));
        });
  }

  // CTRE Phoenix 6 simulation requires periodically updating rotor position/velocity.
  public void updateSim(double dtSeconds) {
    final double batteryVoltage = RobotController.getBatteryVoltage();
    mainSim.setSupplyVoltage(batteryVoltage);
    followSim.setSupplyVoltage(batteryVoltage);

    shooterSim.setInputVoltage(mainSim.getMotorVoltage());
    shooterSim.update(dtSeconds);

    final double rotorPositionRot = shooterSim.getAngularPositionRotations();
    final double rotorVelocityRps =
        RotationsPerSecond.convertFrom(shooterSim.getAngularVelocityRadPerSec(), RadiansPerSecond);

    mainSim.setRawRotorPosition(rotorPositionRot);
    mainSim.setRotorVelocity(rotorVelocityRps);

    followSim.setRawRotorPosition(rotorPositionRot);
    followSim.setRotorVelocity(rotorVelocityRps);
  }
}
