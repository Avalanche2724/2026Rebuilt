package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Spindexer extends SubsystemBase {
  TalonFX mainMotor = new TalonFX(55);

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
          mainMotor.setControl(new VoltageOut(0));
        });
  }
}
