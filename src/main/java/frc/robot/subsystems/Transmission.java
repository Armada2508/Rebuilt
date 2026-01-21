package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;


import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransmissionK;

import static edu.wpi.first.units.Units.RPM;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class Transmission extends SubsystemBase {
 private final Transmission transmission = new Transmission();
    private final TalonSRX conveyor = new TalonSRX(TransmissionK.talonID);
/**
 * The method for running the transmission
 */
    public Transmission() {
        configTalons();
        configMotionMagic(TransmissionK.maxVelocity, TransmissionK.maxAcceleration);
    }
/**
 * Resets then applies the configurations to the motors
 */
    private void configTalons() {
        conveyor.configFactoryDefault();
        
        
        conveyor.setNeutralMode(NeutralMode.Coast);
        conveyor.configPeakOutputForward(0); //! find values
        conveyor.configPeakOutputReverse(0);//! find values
    }
/**
 * sets the conveyer to spin at a positive voltage
 */
    public void spinConveyer() {
        conveyor.set(ControlMode.MotionMagic, -TransmissionK.spinConveyorVoltage);
    }
/**
 * sets conveyer to spin 
 */
     public void spinConveyerNegative() {
        conveyor.set(ControlMode.MotionMagic, -TransmissionK.spinConveyorVoltage);
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
 * Sets motor voltage to zero to stop
 */
    public void stop() {
        conveyor.set(ControlMode.PercentOutput, 0.0);
    }
/**
 * Moves the conveyor back and forth to jostle the balls
 */
    public void jostle() {
        new RunCommand(() -> {
    spinConveyer();
}, transmission)
.withTimeout(.5);
    

    new RunCommand(() -> {
    spinConveyerNegative();
}, transmission)
.withTimeout(.5);
    
new RunCommand(() -> {
    spinConveyer();
}, transmission)
.withTimeout(.5);

    new RunCommand(() -> {
    spinConveyerNegative();
}, transmission)
.withTimeout(.5);
    
}


}




