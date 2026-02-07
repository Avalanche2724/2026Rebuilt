package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {
  private static final int MOTOR_ID = 70;

  private final TalonFX motor = new TalonFX(MOTOR_ID);

  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public Climber() {
    var configs = new TalonFXConfiguration();
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Starting point only; tune on-robot.
    configs.Slot0.kP = 0.0; //15.0;

    motor.getConfigurator().apply(configs);
  }

  public void setPositionRotations(double positionRotations) {
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  public double getPositionRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }

  public Command goToPositionRotations(DoubleSupplier positionRotations) {
    return this.run(() -> setPositionRotations(positionRotations.getAsDouble()));
  }

  public Command runAtVolts(DoubleSupplier volts) {
    return this.startEnd(
        () -> motor.setControl(voltageRequest.withOutput(volts.getAsDouble())),
        () -> motor.setControl(voltageRequest.withOutput(0.0)));
  }
}
