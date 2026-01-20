package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.TransmissionK;
import frc.robot.lib.util.Util;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class Transmission {

    private final TalonSRX conveyor = new TalonSRX(TransmissionK.talonID);

    public Transmission() {
        configTalons();
        configMotionMagic(TransmissionK.maxVelocity, TransmissionK.maxAcceleration);
    }

    private void configTalons() {
        conveyor.configFactoryDefault();
        
        
        conveyor.setNeutralMode(NeutralMode.Coast);
        conveyor.configPeakOutputForward(0); //! find values
        conveyor.configPeakOutputReverse(0);//! find values
    }

    public void spinConveyer() {
        conveyor.set(ControlMode.MotionMagic, TransmissionK.spinConveyorVoltage);
    }

    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(RPM);
        motionMagicConfigs.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond);
        
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.motionAcceleration = 0; //! Find values
        config.motionCruiseVelocity = 0;

        conveyor.configAllSettings(config);
    }


    public void stop() {
        conveyor.set(ControlMode.PercentOutput, 0.0);
    }

    public void jostle() {
        
    }
}

