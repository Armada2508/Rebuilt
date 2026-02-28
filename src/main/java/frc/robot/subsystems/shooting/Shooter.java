package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
import frc.robot.lib.util.Util;

@Logged
public class Shooter extends SubsystemBase {

    private final TalonFX talonFlywheelLeft = new TalonFX(ShooterK.talonShooterLeftID); // As viewed from the back of the turret structure
    private final TalonFX talonFlywheelRight = new TalonFX(ShooterK.talonShooterRightID); // As viewed from the back of the turret structure
    private final TalonFX talonHood = new TalonFX(ShooterK.talonHoodID);

    
    
    public Shooter() {
        configTalons();
        configMotionMagic();
    }

    /**
     * Configures the motors
     * talonFlywheelLeft NeutralMode needs to be set as coastMode for the longevity of the motor
     * (brakeMode may cause damage in a fast moving motor)
     */
    public void configTalons() {
        Util.factoryReset(talonFlywheelLeft, talonFlywheelRight, talonHood);
        Util.coastMode(talonFlywheelLeft, talonFlywheelRight);
        Util.brakeMode(talonHood);

        talonFlywheelRight.setControl(new StrictFollower(talonFlywheelLeft.getDeviceID()));

        MotorOutputConfigs invertConfig = new MotorOutputConfigs();
        invertConfig.Inverted = InvertedValue.Clockwise_Positive;

        talonFlywheelRight.getConfigurator().apply(invertConfig);
        talonFlywheelLeft.getConfigurator().apply(ShooterK.shooterCurrentLimitsConfigs);
        talonFlywheelLeft.getConfigurator().apply(ShooterK.flywheelPidConfig);

        talonHood.getConfigurator().apply(ShooterK.hoodPidConfig);
        talonHood.getConfigurator().apply(ShooterK.hoodSoftwareLimitSwitchConfig);
        talonHood.getConfigurator().apply(ShooterK.hoodCurrentLimitsConfigs);
        talonHood.getConfigurator().apply(ShooterK.gearRatioConfig);
    }

    /**
     * Configures MotionMagic and applies it to talonHood
     */
    public void configMotionMagic() {
        // System.out.println("Motion Magic Configuring");
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs()
        .withMotionMagicAcceleration(ShooterK.motionMagicAcceleration);
        // .withMotionMagicCruiseVelocity(ShooterK.motionMagicVelocity);
        talonFlywheelLeft.getConfigurator().apply(motionMagicConfig);
    }

    //public Command setShooterVoltage(Voltage voltage) {
    //    return runOnce(() -> {
    //        talonFlywheelLeft.setControl(new VoltageOut(voltage.in(Volts)));
    //    }).withName("Set Shooter Voltage");
    //}
    //^ I don't think we're using voltage to control the shooter so I don't believe this is needed

    /**
     * Returns the velocity in rpm of the shooting motor
     * @return
     */
    public AngularVelocity getMotorVelocity() {
        return talonFlywheelLeft.getVelocity().getValue().times(60);
    }

    /**
     * Sets the shooter to a set RPM
     * @param rpm The RPM to shoot at
     * @return
     */
    public Command setShooterVelocity(AngularVelocity rpm) {
        return runOnce(() -> talonFlywheelLeft.setControl(new VelocityVoltage(rpm)));
    }

    public void shoot() {
        // System.out.println("shoot method called");
        MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(ShooterK.staticRpm);
        talonFlywheelLeft.setControl(request);
        // System.out.println("Control request set");

        // final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        // talonFlywheelLeft.setControl(request.withVelocity(ShooterK.staticRpm));

        // talonFlywheelLeft.setVoltage(ShooterK.shooterVoltage.in(Volts));
    }

    /**
     * Shoots the fuel at a static RPM
     * @return
     */
    public Command shootFuel() {
        return runOnce(() -> shoot())
        .withName("Shoot Fuel");
    }

    /**
     * Sets the hood to a target angle using Motion Magic
     * @param targetAngle Angle to set the hood to
     * @return runnable containing a command to command the talon
     */
    public Command setHoodAngle(Angle targetAngle) {
        //return runOnce(() -> sparkMaxController.setSetpoint(targetAngle.in(Degrees), ControlType.kMAXMotionPositionControl));
        MotionMagicVoltage request = new MotionMagicVoltage(targetAngle);
        return runOnce(() -> talonHood.setControl(request))
        .withName("Set Hood Angle");
    }

    /**
     * Sets the hood to its minimum angle
     */
    public Command stow() {
        return runOnce(() -> talonHood.setPosition(ShooterK.minHoodAngle))
        .withName("Stow");
    }

    /**
     * Stops the shooter and the hood motors from moving
     */
    public Command stop() {
        return runOnce(() -> talonFlywheelLeft.setControl(new NeutralOut()))
        .andThen(runOnce(() -> talonHood.setControl(new NeutralOut())))
        .withName("Stop");
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
}
