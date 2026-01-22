package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;


import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransmissionK;

import static edu.wpi.first.units.Units.RPM;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class Transmission extends SubsystemBase {
    private final TalonSRX conveyor = new TalonSRX(TransmissionK.talonID);
    /**
     * The method for running the transmission
     */
    public Transmission() {
        configTalons();
        configMotionMagic(TransmissionK.maxVelocity, TransmissionK.maxAcceleration);
    }
    /**
     * configures the motion magic for the motors
     * @param velocity
     * @param acceleration
     */
    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(RPM);
        motionMagicConfigs.MotionMagicAcceleration = acceleration.in(RotationsPerSecondPerSecond);
        
        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.motionAcceleration = 0; //! Find values
        config.motionCruiseVelocity = 0;

        conveyor.configAllSettings(config);
    }
    /**
     * Resets then applies the configurations to the motors
     */
    private void configTalons() {
        conveyor.configFactoryDefault();
        
        conveyor.setNeutralMode(NeutralMode.Coast);
    }
    /**
     * sets the conveyer to spin forward
     */
    public void spinConveyer() {
        conveyor.set(ControlMode.MotionMagic, TransmissionK.spinConveyorVoltage);
    }
    /**
     * sets conveyer to spin backwards
     */
     public void spinConveyerNegative() {
        conveyor.set(ControlMode.MotionMagic, -TransmissionK.spinConveyorVoltage);
    }
    /**
     * Moves the conveyor back and forth to jostle the fuel
     */
    public void jostle() {
        runOnce(() -> {
            spinConveyer();
    })
            .withTimeout(.5);
    
        
        runOnce(() -> {
            spinConveyerNegative();
    })
            .withTimeout(.5);
    
        
        runOnce(() -> {
            spinConveyer();
    })
            .withTimeout(.5);

        
        runOnce(() -> {
            spinConveyerNegative();
    })
            .withTimeout(.5);
}
    /** 
     * Sets motor voltage to zero for stopping
     */
    public void stop() {
        conveyor.neutralOutput();
    }
}




