package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;

@Logged
public class Intake extends SubsystemBase{

    private final SparkMax extender = new SparkMax(IntakeK.extenderID, MotorType.kBrushless);
    private final SparkMax wheels = new SparkMax(IntakeK.wheelsID, MotorType.kBrushless);

    public Intake() {
        configSparkMaxs();
    }
    
    private void configSparkMaxs() {
        SparkMaxConfig extenderConfig = new SparkMaxConfig();
        SparkMaxConfig wheelsConfig = new SparkMaxConfig();
        
        extenderConfig.idleMode(IdleMode.kCoast);
        extenderConfig.smartCurrentLimit(IntakeK.extenderCurrentLimit);

        wheelsConfig.idleMode(IdleMode.kBrake);
        wheelsConfig.smartCurrentLimit(IntakeK.wheelsCurrentLimit);

        extenderConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true).warningsAlwaysOn(true).faultsAlwaysOn(true);
        extender.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wheelsConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true).warningsAlwaysOn(true).faultsAlwaysOn(true);
        wheels.configure(wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
         
    }

    /**
     * Declares that it is a voltage output for both extender and wheels
     * @param volts
     * @return
     */
    private Command setVoltage(Voltage volts) {
        return runOnce(() -> {
            extender.setVoltage(volts);
            wheels.setVoltage(volts);
        })
        .withName("Set Voltage");
    }
    
    /**
     * Extends the arm mechanism by setting the voltage using .extendVoltage
     * @return
     */
    public Command extend() { 
        return runOnce(() -> {
            setVoltage(IntakeK.extendVoltage);
       })
       .withName("Extended");
    }

    /**
     * Retracts the arm mechanism by setting the voltage to negative
     * @return
     */
    public Command retract() {
        return runOnce(() -> {
            setVoltage(IntakeK.extendVoltage.unaryMinus());
        })
        .withName("Retracted");
    }

    /**
     * Spins intake wheels/motors via spinWheelsVoltage
     */
    public void spinWheels() { 
        wheels.setVoltage(IntakeK.spinWheelsVoltage); //! find value
    }

    /**
     * Sets the intake wheels to stop
     */
    public Command stopWheels() {
        return runOnce(() -> {
            wheels.stopMotor();
        })
        .withName("Stop Wheels");
    }

    /**
     * Stops both motors
     */
    public void stop() { 
        extender.stopMotor();
        wheels.stopMotor();
    }
}
