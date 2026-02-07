package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Spindexer extends SubsystemBase {
  private static final int MAIN_MOTOR_ID = 55;

  private final TalonFX mainMotor = new TalonFX(MAIN_MOTOR_ID);

  private final TalonFXSimState mainSim = mainMotor.getSimState();
  private final DCMotorSim spindexerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1),
              0.0002, // moment of inertia (kg*m^2), tune as needed
              1.0 // output-to-input (rotor) ratio
              ),
          DCMotor.getKrakenX60Foc(1));

  public Spindexer() {
    var configs = new TalonFXConfiguration();
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    mainMotor.getConfigurator().apply(configs);
  }

  public Command runAtVolts(DoubleSupplier volts) {
    return this.startEnd(
        () -> {
          mainMotor.setControl(new VoltageOut(volts.getAsDouble()));
        },
        () -> {
          mainMotor.setControl(new VoltageOut(0.0));
        });
  }

  @SuppressWarnings("DuplicatedCode")
  public void updateSim(double dtSeconds) {
    double batteryVoltage = RobotController.getBatteryVoltage();
    mainSim.setSupplyVoltage(batteryVoltage);

    spindexerSim.setInputVoltage(mainSim.getMotorVoltage());
    spindexerSim.update(dtSeconds);

    double rotorPositionRot = spindexerSim.getAngularPositionRotations();
    double rotorVelocityRps =
        RotationsPerSecond.convertFrom(
            spindexerSim.getAngularVelocityRadPerSec(), RadiansPerSecond);

    mainSim.setRawRotorPosition(rotorPositionRot);
    mainSim.setRotorVelocity(rotorVelocityRps);
  }
}
