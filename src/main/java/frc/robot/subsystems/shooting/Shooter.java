package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

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

    //* https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/cancoder/index.html
    private final CANcoder canCoder = new CANcoder(9);
    
    public Shooter() {
        configTalons();
        configMotionMagic();
        configCanCoder();

        canCoder.setPosition(0); //^ Zero the hood encoder on startup
    }

    /**
     * Configures the motors
     * talonFlywheelLeft NeutralMode needs to be set as coastMode for the longevity of the motor
     * (brakeMode may cause damage in a fast moving motor)
     */
    private void configTalons() {
        Util.factoryReset(talonFlywheelLeft, talonFlywheelRight, talonHood);
        Util.coastMode(talonFlywheelLeft, talonFlywheelRight, talonHood);
        // Util.brakeMode(talonHood);

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
    private void configMotionMagic() {
        MotionMagicConfigs motionMagicFlywheelConfig = new MotionMagicConfigs()
        .withMotionMagicAcceleration(ShooterK.motionMagicFlywheelAcceleration);
        talonFlywheelLeft.getConfigurator().apply(motionMagicFlywheelConfig);

        MotionMagicConfigs motionMagicHoodConfig = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(ShooterK.motionMagicHoodVelocity)
        .withMotionMagicAcceleration(ShooterK.motionMagicHoodAcceleration);
        talonHood.getConfigurator().apply(motionMagicHoodConfig);
    }

    private void configCanCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor = new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(0) //! Find
        .withMagnetOffset(0) //~ Might not be needed?, Find
        // We might be able to set the offset via TunerX
        .withSensorDirection(null); //! Find


    }

    /**
     * Returns the velocity in rpm of the shooting motor
     * @return
     */
    @Logged(name = "Motor Velocity (rpm)")
    public double getMotorVelocity() {
        return talonFlywheelLeft.getVelocity().getValue().in(RotationsPerSecond) * 60;
    }

    /**
     * Returns the angle of the hood as read by the CANCoder
     * @return The angle of the hood in degrees
     */
    @Logged(name = "Hood Angle (degrees)")
    public Angle getHoodAngle() {
        return Degrees.of(canCoder.getAbsolutePosition().getValue().in(Rotations)); //! Test
    }

    /**
     * Convert a measure of [0, 1) rotations into [0, 360) degrees
     * @param rotations
     * @return
     */
    //! This method COULD be helpful in the future
    // public Angle asDegrees(double rotations) {
    //     return Degrees.of(rotations * 360);
    // }

    /**
     * Sets the shooter to a set RPM
     * @param rpm The RPM to shoot at
     * @return
     */
    public Command setShooterVelocity(AngularVelocity rpm) {
        return runOnce(() -> talonFlywheelLeft.setControl(new VelocityVoltage(rpm)));
    }

    public void shoot() {
        MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(ShooterK.staticRpm);
        talonFlywheelLeft.setControl(request);
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
