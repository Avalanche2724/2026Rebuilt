package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    TalonFX motor1 = new TalonFX(41);
    TalonFX motor2 = new TalonFX(42);

    public Command spinMotor() {
        return runOnce(() -> {

            motor1.setVoltage(.5);
        });
    }
}
