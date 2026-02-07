package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Utils;
import java.util.function.DoubleSupplier;

public class Spindexer extends SubsystemBase {
  private static final int MAIN_MOTOR_ID = 55;

  private final TalonFX mainMotor = new TalonFX(MAIN_MOTOR_ID);

  private final StatusSignal<AngularVelocity> mainMotorVelocityRps = mainMotor.getVelocity(false);
  private final StatusSignal<Current> mainMotorTorqueCurrentAmps =
      mainMotor.getTorqueCurrent(false);

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

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
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    Utils.applyTalonFxConfigWithRetry("Spindexer/Main", mainMotor, configs);

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, mainMotorVelocityRps, mainMotorTorqueCurrentAmps);

    setDefaultCommand(brakeSpindexer());

    if (RobotBase.isSimulation()) {
      mainSim.Orientation = ChassisReference.CounterClockwise_Positive;
      mainSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }
  }

  public Command runAtVolts(DoubleSupplier volts) {
    return this.runEnd(
        () -> mainMotor.setControl(voltageRequest.withOutput(volts.getAsDouble())),
        () -> mainMotor.setControl(voltageRequest.withOutput(0.0)));
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(mainMotorVelocityRps, mainMotorTorqueCurrentAmps);
  }

  public void updateSim(double dtSeconds) {
    Utils.updateTalonFxRotorSim(dtSeconds, spindexerSim, mainSim);
  }

  private Command brakeSpindexer() {
    return this.run(() -> mainMotor.setControl(neutralRequest));
  }
}
