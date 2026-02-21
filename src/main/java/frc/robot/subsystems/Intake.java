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
    private final SparkMax roller = new SparkMax(IntakeK.rollerID, MotorType.kBrushless);
    
    public Intake() {
        configSparkMaxs();
    }
    
    private void configSparkMaxs() {
        SparkMaxConfig extenderConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(IntakeK.rollerCurrentLimit);
        rollerConfig.signals.primaryEncoderPositionAlwaysOn(true).primaryEncoderVelocityAlwaysOn(true).warningsAlwaysOn(true).faultsAlwaysOn(true);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    
    private Command setRollerVoltage(Voltage rollerVolts) {
        return runOnce(() -> {
            roller.setVoltage(rollerVolts);
        })
        .withName("Set roller Voltage");
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
     * Spins intake roller/motors via spinrollerVoltage
     */
    public Command spinRoller() { 
        return runOnce(() -> {
            setRollerVoltage(IntakeK.spinRollerVoltage);
        })
        .withName("Spinning roller");
    }

    /**
     * Sets the intake roller to stop
     */
    public Command stopRoller() {
        return runOnce(() -> {
            roller.stopMotor();
        })
        .withName("Stop roller");
    }

    /**
     * Stops both motors
     */
    public void stop() { 
        extender.stopMotor();
        roller.stopMotor();
    }

    /**
     * Returns the currently running command
     * @return The command being run
     */
    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }

    /**
     * Returns the applied voltage to the extender motor
     * @return The applied voltage
     */
    @Logged(name = "Extender Voltage (v)")
    public double getExtenderVoltage() {
        return extender.getAppliedOutput();
    }

    /**
     * Returns the applied voltage to the roller motor
     * @return The applied voltage
     */
    @Logged(name = "Roller Voltage (v)")
    public double getRollerVoltage() {
        return roller.getAppliedOutput();
    }
}
