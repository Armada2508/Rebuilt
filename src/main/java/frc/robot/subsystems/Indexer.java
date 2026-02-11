package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerK;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Indexer extends SubsystemBase {
    private final TalonSRX indexer = new TalonSRX(IndexerK.talonID);

    /**
     * The method for running the indexer
     */
    public Indexer() {
        configTalons();
    }

    /**
     * Resets then applies the configurations to the motors
     */
    private void configTalons() {
        indexer.configFactoryDefault();
        indexer.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * sets the Indexer to spin forward
     */
    public void spinIndexer() {
        //indexer.set(ControlMode.MotionMagic, IndexerK.spinIndexerVoltage.in(Volts));
        indexer.set(ControlMode.Velocity, IndexerK.spinindexerVoltage.in(Volts));
    }

    /**
     * sets Indexer to spin backwards
     */
     public void spinIndexerNegative() {
        //indexer.set(ControlMode.MotionMagic, -IndexerK.spinIndexerVoltage.in(Volts));
        indexer.set(ControlMode.Velocity, -IndexerK.spinindexerVoltage.in(Volts));
    }

    /**
     * Moves the indexer back and forth to jostle the fuel
     */
    public Command jostle() {
       return runOnce(() -> {
            spinIndexer();
    })
        .withTimeout(IndexerK.jostleDuration)
        .andThen(() -> {
                spinIndexerNegative();
    })
        .withTimeout(IndexerK.jostleDuration)
        .andThen(() -> {
                spinIndexer();
    })
        .withTimeout(IndexerK.jostleDuration)
        .andThen(() -> {
                spinIndexerNegative();
    })
        .withTimeout(IndexerK.jostleDuration);
    }

    /** 
     * Sets motor to neutral so it stops running
     */
    public void stop() {
        indexer.neutralOutput();
    }
}




