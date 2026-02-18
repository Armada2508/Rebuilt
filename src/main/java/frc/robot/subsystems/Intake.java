package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Inches;

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
public class Intake extends SubsystemBase {


    private final SparkMax extender = new SparkMax(IntakeK.extenderID, MotorType.kBrushless);
    private final SparkMax wheels = new SparkMax(IntakeK.wheelsID, MotorType.kBrushless);
    
    public Intake() {
        configSparkMaxs();
    }
    
    private void configSparkMaxs() {
        SparkMaxConfig extenderConfig = new SparkMaxConfig();
        SparkMaxConfig wheelsConfig = new SparkMaxConfig();
        
        wheelsConfig.idleMode(IdleMode.kCoast);
        wheelsConfig.smartCurrentLimit(IntakeK.wheelsCurrentLimit);
        wheelsConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true).warningsAlwaysOn(true).faultsAlwaysOn(true);
        wheels.configure(wheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Limit switch/soft limit for arm
        extenderConfig.limitSwitch.forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor); //! check
        extenderConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
        extenderConfig.softLimit.forwardSoftLimit((IntakeK.forwardSoftLimit.in(Inches))).reverseSoftLimit(IntakeK.reverseSoftLimit.in(Inches)).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
        extender.configure(extenderConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }
    
    /**
     * Declares that it is a voltage output for extender
     * @param volts
     * @return
     */
    private Command setExtenderVoltage(Voltage volts) {
        return runOnce(() -> {
            extender.setVoltage(volts);
        })
        .withName("Set Extender Voltage");
    }
    
    private Command setWheelsVoltage(Voltage wheelsVolts) {
        return runOnce(() -> {
            wheels.setVoltage(wheelsVolts);
        })
        .withName("Set Wheels Voltage");
    }

    /**
     * Extends the arm mechanism by setting the voltage using .extendVoltage
     * @return
     */
    public Command extend() { 
        return runOnce(() -> {
            setExtenderVoltage(IntakeK.extendVoltage);
       })
       .withName("Extended");
    }

    /**
     * Retracts the arm mechanism by setting the voltage to negative
     * @return
     */
    public Command retract() {
        return runOnce(() -> {
            setExtenderVoltage(IntakeK.extendVoltage.unaryMinus());
        })
        .withName("Retracted");
    }

    /**
     * Spins intake wheels/motors via spinWheelsVoltage
     */
    public Command spinWheels() { 
        return runOnce(() -> {
            setWheelsVoltage(IntakeK.spinWheelsVoltage);
        })
        .withName("Spinning Wheels");
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
