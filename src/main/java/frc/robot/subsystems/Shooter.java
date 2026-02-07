package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

public class Shooter extends SubsystemBase {
  private static final int MAIN_MOTOR_ID = 61;
  private static final int FOLLOW_MOTOR_ID = 62;

  private final TalonFX mainMotor = new TalonFX(MAIN_MOTOR_ID);
  private final TalonFX followMotor = new TalonFX(FOLLOW_MOTOR_ID);

  private final StatusSignal<AngularVelocity> mainMotorVelocityRps = mainMotor.getVelocity(false);
  private final StatusSignal<Current> mainMotorTorqueCurrentAmps =
      mainMotor.getTorqueCurrent(false);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final CoastOut coastRequest = new CoastOut();

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

    Utils.applyTalonFxConfigWithRetry("Shooter/Main", mainMotor, configs);
    Utils.applyTalonFxConfigWithRetry("Shooter/Follow", followMotor, configs);

    followMotor.setControl(new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        250.0, mainMotorVelocityRps, mainMotorTorqueCurrentAmps);

    setDefaultCommand(coastShooter());

    if (RobotBase.isSimulation()) {
      mainSim.Orientation = ChassisReference.CounterClockwise_Positive;
      followSim.Orientation = ChassisReference.CounterClockwise_Positive;
      mainSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
      followSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }
  }

  public Command runAtSpeed(DoubleSupplier speedRps) {
    return this.runEnd(
        () -> mainMotor.setControl(velocityRequest.withVelocity(speedRps.getAsDouble())),
        () -> mainMotor.setControl(voltageRequest.withOutput(0.0)));
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(mainMotorVelocityRps, mainMotorTorqueCurrentAmps);
  }

  public void updateSim(double dtSeconds) {
    Utils.updateTalonFxRotorSim(dtSeconds, shooterSim, mainSim, followSim);
  }

  private Command coastShooter() {
    return this.run(() -> mainMotor.setControl(coastRequest));
  }
}
