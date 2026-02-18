package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;

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
        
        extenderConfig.idleMode(IdleMode.kBrake);
        extenderConfig.smartCurrentLimit(IntakeK.extenderCurrentLimit);

        wheelsConfig.idleMode(IdleMode.kCoast);
        wheelsConfig.smartCurrentLimit(IntakeK.wheelsCurrentLimit);

        extenderConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true).warningsAlwaysOn(true).faultsAlwaysOn(true);
        extender.configure(extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wheelsConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true).warningsAlwaysOn(true).faultsAlwaysOn(true);
        wheels.configure(wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig config = new SparkMaxConfig();

    config.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor); //! check
    config.limitSwitch.forwardLimitSwitchType(
        LimitSwitchConfig.Type.kNormallyOpen
    );

    extender.configure(config, SparkMax.ResetMode.kResetSafeParameters,
                        SparkMax.PersistMode.kPersistParameters);
}
    

    /**
     * Declares that it is a voltage output for extender
     * @param volts
     * @return
     */
    private Command setVoltageExtender(Voltage volts) {
        return runOnce(() -> {
            extender.setVoltage(volts);
            wheels.setVoltage(volts);
        })
        .withName("Set Extender Voltage");
    }
    /**
     * Declares that it is a voltage output for wheels
     * @param volts
     * @return
     */
    private Command setVoltageWheels(Voltage volts) {
        return runOnce(() -> {
            extender.setVoltage(volts);
            wheels.setVoltage(volts);
        })
        .withName("Set Wheels Voltage");
    }
    
    /**
     * Extends the arm mechanism by setting the voltage using .extendVoltage
     * @return
     */
    public Command extend() { 
        return runOnce(() -> {
            setVoltageExtender(IntakeK.extendVoltage);
       })
       .withName("Extended");
    }

    /**
     * Retracts the arm mechanism by setting the voltage to negative
     * @return
     */
    public Command retract() {
        return runOnce(() -> {
            setVoltageWheels(IntakeK.extendVoltage.unaryMinus());
        })
        .withName("Retracted");
    }

    /**
     * Spins intake wheels/motors via spinWheelsVoltage
     */

    public Command spinWheels() { 
        return runOnce(() -> {
            wheels.setVoltage(IntakeK.spinWheelsVoltage); //! find value
        })
        .withName("Spinning");
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
