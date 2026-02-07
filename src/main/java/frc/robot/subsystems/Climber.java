package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Utils;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {
  private static final int MOTOR_ID = 58;

  private final TalonFX motor = new TalonFX(MOTOR_ID);

  private final StatusSignal<Angle> positionRotations = motor.getPosition(false);
  private final StatusSignal<Current> torqueCurrentAmps = motor.getTorqueCurrent(false);

  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public Climber() {
    var configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Starting point only; tune on-robot.
    configs.Slot0.kP = 0.0; // 15.0;

    Utils.applyTalonFxConfigWithRetry("Climber/Motor", motor, configs);

    BaseStatusSignal.setUpdateFrequencyForAll(250.0, positionRotations, torqueCurrentAmps);

    setDefaultCommand(stopClimber());
  }

  private void setPositionRotations(double positionRotations) {
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  public double getPositionRotations() {
    return positionRotations.getValueAsDouble();
  }

  private void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }

  public Command goToPositionRotations(DoubleSupplier positionRotations) {
    return this.run(() -> setPositionRotations(positionRotations.getAsDouble()));
  }

  public Command runAtVolts(DoubleSupplier volts) {
    return this.runEnd(
        () -> motor.setControl(voltageRequest.withOutput(volts.getAsDouble())), this::stop);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionRotations, torqueCurrentAmps);
  }

  private Command stopClimber() {
    return this.run(this::stop);
  }
}
