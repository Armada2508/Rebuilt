package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerK;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class Indexer extends SubsystemBase {
    private final TalonSRX conveyor = new TalonSRX(IndexerK.talonID);

    /**
     * The method for running the Indexer
     */
    public Indexer() {
        configTalons();
        configMotionMagic(IndexerK.maxRPM, IndexerK.maxAcceleration);
    }

    /**
     * configures the motion magic for the motors
     * @param velocity
     * @param acceleration
     */
    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
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
        conveyor.set(ControlMode.MotionMagic, IndexerK.spinConveyorVoltage.in(Volts));
    }

    /**
     * sets conveyer to spin backwards
     */
     public void spinConveyerNegative() {
        conveyor.set(ControlMode.MotionMagic, -IndexerK.spinConveyorVoltage.in(Volts));
    }

    /**
     * Moves the conveyor back and forth to jostle the fuel
     */
    public Command jostle() {
       return runOnce(() -> {
            spinConveyer();
    })
        .withTimeout(IndexerK.jostleDuration)
        .andThen(() -> {
                spinConveyerNegative();
    })
        .withTimeout(IndexerK.jostleDuration)
        .andThen(() -> {
                spinConveyer();
    })
        .withTimeout(IndexerK.jostleDuration)
        .andThen(() -> {
                spinConveyerNegative();
    })
        .withTimeout(IndexerK.jostleDuration);
    }

    /** 
     * Sets motor to neutral so it stops running
     */
    public void stop() {
        conveyor.neutralOutput();
    }
}




