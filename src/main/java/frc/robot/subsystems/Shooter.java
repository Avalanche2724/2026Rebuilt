package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    TalonFX mainMotor = new TalonFX(61);
    TalonFX followMotor = new TalonFX(62);

    public Shooter() {
        var configs = new TalonFXConfiguration();
        configs.CurrentLimits.StatorCurrentLimit = 200;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 120;
        configs.CurrentLimits.SupplyCurrentLowerTime = 2;
        configs.CurrentLimits.SupplyCurrentLowerLimit = 40;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        followMotor.setControl(new Follower(mainMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        mainMotor.getConfigurator().apply(configs);
        followMotor.getConfigurator().apply(configs);

        mainMotor.getMotorVoltage().setUpdateFrequency(250);
        mainMotor.getTorqueCurrent().setUpdateFrequency(250);
        mainMotor.getVelocity().setUpdateFrequency(250);
        mainMotor.getSupplyCurrent().setUpdateFrequency(250);
        mainMotor.getSupplyVoltage().setUpdateFrequency(250);

    }
}
