package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;
import frc.robot.lib.util.Encoder;

@Logged
public class Intake extends SubsystemBase {

    private final SparkMax extender = new SparkMax(IntakeK.extenderID, MotorType.kBrushless);
    private final SparkMax roller = new SparkMax(IntakeK.rollerID, MotorType.kBrushless);
    
    public Intake() {
        configSparkMaxs();
        extender.getEncoder().setPosition(0); // Zero the encoder on startup
        //! Test today, does this command it to go to a position or reset its position?
    }

    @Override
    public void periodic() {
        if (getExtenderCurrent() > 24) {
            extender.stopMotor();
            System.out.println("Stopping extender CURRENT | " + getExtenderCurrent());
        }
        if (getArmPosition() > 0.1 || getArmPosition() < -18) {
            extender.stopMotor();
            System.out.println("Stopping Extender POSITION | " + getArmPosition());
        }
    }

    @SuppressWarnings("removal")
    private void configSparkMaxs() {
        SparkMaxConfig extenderConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        
        //~ Roller Config
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(IntakeK.rollerCurrentLimit);

        rollerConfig.signals //& Roller Signals
        // .primaryEncoderPositionAlwaysOn(true) //? I don't believe we care about this
        .primaryEncoderVelocityAlwaysOn(true)
        .warningsAlwaysOn(true)
        .faultsAlwaysOn(true);

        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //~ Extender Config
        extenderConfig.idleMode(IdleMode.kBrake);
        extenderConfig.smartCurrentLimit(IntakeK.extenderCurrentLimit);

        extenderConfig.signals //& Extender Signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .warningsAlwaysOn(true)
        .faultsAlwaysOn(true);

        // extenderConfig.encoder //& Extender Encoder
        // .positionConversionFactor(IntakeK.extenderGearRatio); // Apply conversion for encoder, rotations -> inches

        // Limit switch/soft limit for arm
        // extenderConfig.limitSwitch //& Extender Limit Switch 
        //                           //? Is this even being used right now?
        // .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor) //! check
        // .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

        // extenderConfig.softLimit //& Extender Soft Limit
        // .forwardSoftLimitEnabled(true)
        // .reverseSoftLimitEnabled(true)
        // .forwardSoftLimit((IntakeK.forwardSoftLimit.in(Inches)))
        // .reverseSoftLimit(IntakeK.reverseSoftLimit.in(Inches));

        extender.configure(extenderConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    /**
     * Extends the arm mechanism by setting the voltage using .extendVoltage
     * @return
     */
    public Command extend() { 
    //     return runOnce(() -> {
    //         extender.setVoltage(IntakeK.extendVoltage);
    //    }).alongWith(Commands.waitUntil(() -> getExtenderCurrent() > 24)
    //    .andThen(Commands.runOnce(() -> extender.stopMotor())))
    //    .withName("Extend");

        return new RepeatCommand(Commands.runOnce(() -> extender.setVoltage(IntakeK.extendVoltage)))
        // .until(() -> (getExtenderCurrent() > 24))
        .withName("Extend");
    }

    /**
     * Retracts the arm mechanism by setting the voltage to negative
     * @return
     */
    public Command retract() {
    //     return runOnce(() -> {
    //         extender.setVoltage(IntakeK.retractVoltage);
    //    }).alongWith(Commands.waitUntil(() -> getExtenderCurrent() > 24)
    //    .andThen(Commands.runOnce(() -> extender.stopMotor())))
    //    .withName("Retract");

        return new RepeatCommand(Commands.runOnce(() -> extender.setVoltage(IntakeK.retractVoltage)))
        .withName("Extend");
    }

    /**
     * Spins intake roller/motors via spinrollerVoltage
     */
    public Command spinRoller() { 
        return runOnce(() -> 
            roller.setVoltage(IntakeK.spinRollerVoltage)
        )
        .withName("Spin roller");
    }

    public Command spinRollerReverse() { 
        return runOnce(() -> 
            roller.setVoltage(IntakeK.spinRollerReverseVoltage)
        )
        .withName("Spin roller");
    }

    public Command stopArm() { //! Check if we need this
        return runOnce(() -> {
            extender.stopMotor();
        })
        .withName("Stop Arm");
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
     * Re-zeros the extender
     * ! Do not call this in the middle of a match unless you aboslutely have to
     */
    public void zeroExtender() {
        extender.getEncoder().setPosition(0);
    }

    /**
     * Stops both motors
     */
    public void stop() { 
        extender.stopMotor();
        roller.stopMotor();
    }

    // @Override
    // public void periodic() {
    //     if (getArmPosition().gt(Inches.of(8)) || getArmPosition().lt(Inches.of(8))) stopArm(); //! find
    // }

    @Logged(name = "Arm Position (In)")
    public double getArmPosition() {
        // return Encoder.angularToLinear(
        //     Rotations.of(extender.getEncoder().getPosition()),
        //     IntakeK.extenderGearRatio,
        //     IntakeK.extenderWheelDiameter
        // );
        return extender.getEncoder().getPosition() * IntakeK.extenderGearRatio;
    }

    @Logged(name = "Extender Rotations")
    public double getExtenderRotations() {
        return extender.getEncoder().getPosition();
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
        return extender.getBusVoltage();
    }

    @Logged(name = "Extender Current (amps)")
    public double getExtenderCurrent() {
        return extender.getOutputCurrent();
    }

    /**
     * Returns the applied voltage to the roller motor
     * @return The applied voltage
     */
    @Logged(name = "Roller Voltage (v)")
    public double getRollerVoltage() {
        return roller.getBusVoltage();
    }

    /**
     * Returns the velocity of the roller in Rpm
     * @return The rpm
     */
    @Logged(name = "Roller Velocity (rpm)")
    public double getRollerRpm() {
        return roller.getEncoder().getVelocity();
    }
}


