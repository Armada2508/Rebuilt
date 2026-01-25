package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;
import frc.robot.lib.util.Encoder;
import frc.robot.lib.util.Util;

@Logged
public class Intake extends SubsystemBase{

    private final TalonFX wheels = new TalonFX(IntakeK.wheelsID);
    private final TalonFX extension = new TalonFX(IntakeK.extensionID);

    public Intake() {
        configTalons();
    }

    /**
     * factory resets motors, brake modes, applies PID, software limits, and current limits
     */
    private void configTalons() {
        Util.factoryReset(wheels, extension);
        Util.brakeMode(wheels);
        Util.coastMode(extension);

        // wheels.getConfigurator().apply(IntakeK.softwareLimitConfig);
        wheels.getConfigurator().apply(IntakeK.wheelCurrentConfigs);

        extension.getConfigurator().apply(IntakeK.softwareLimitConfigs);
        extension.getConfigurator().apply(IntakeK.talonCurrentConfigs);
    }

    /**
     * Sets voltage of the talon motor
     * @param volts
     */
    private void setVoltage(Voltage volts) {
        extension.setControl(new VoltageOut(volts)); //! find
    }
    
    /**
     * Extends the arm mechanism by setting the voltage
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
     * Gets current position of intake arm
     * @return
     */
    public Distance getPosition() { //! This probably doesnt work
        return Encoder.angularToLinear(extension.getPosition().getValue(), IntakeK.wheelDiameter);
    }

    /**
     * Spins intake wheels/motors via voltage
     */
    public void spinWheels() { 
        wheels.setControl(new VoltageOut(IntakeK.spinWheelsVoltage)); //! find value
    }

    /**
     * Sets the intake wheels to stop by neutraling the output
     */
    public Command stopWheels() {
        return runOnce(() -> {
            wheels.setControl(new NeutralOut());
        })
        .withName("Stop Wheels");
    }

    /**
     * Stops motors
     */
    public void stop() { 
        extension.setControl(new NeutralOut());
        wheels.setControl(new NeutralOut());
    }
}
