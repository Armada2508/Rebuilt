package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
import frc.robot.lib.util.Util;

@Logged
public class Shooter extends SubsystemBase {

    private final TalonFX talonFlywheelLeft = new TalonFX(ShooterK.talonShooterLeftID); // As viewed from the back of the turret structure
    private final TalonFX talonFlywheelRight = new TalonFX(ShooterK.talonShooterRightID); // As viewed from the back of the turret structure
    private final TalonFX talonHood = new TalonFX(ShooterK.talonHoodID);

    //* https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/cancoder/index.html
    private final CANcoder canCoder = new CANcoder(ShooterK.CANCoderID); 
    
    
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

        talonHood.getConfigurator().apply(invertConfig);
        talonHood.getConfigurator().apply(ShooterK.hoodPidConfig);
        // talonHood.getConfigurator().apply(ShooterK.hoodSoftwareLimitSwitchConfig);
        talonHood.getConfigurator().apply(ShooterK.hoodCurrentLimitsConfigs);
        talonHood.getConfigurator().apply(ShooterK.feedBackConfig);
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
        .withAbsoluteSensorDiscontinuityPoint(1)
        .withMagnetOffset(-0.016)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive); 
        canCoder.getConfigurator().apply(config);
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
    public double getHoodAngle() {
        // return canCoder.getAbsolutePosition().getValue();
        double theta = canCoder.getAbsolutePosition().getValue().times(ShooterK.encoderToHoodGearRatio).in(Degrees);
        if (theta > ShooterK.maxHoodAngle.in(Degrees) + 0.5 || theta < ShooterK.minHoodAngle.in(Degrees)) theta = 0;
        return theta;
        // return Degrees.of(angle);
    }


    public Command shoot(Supplier<AngularVelocity> target) {
        return runOnce(() -> {
            AngularVelocity rpm = target.get();

            MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(rpm);
            SmartDashboard.putNumber("target rpm", rpm.in(RPM));

            talonFlywheelLeft.setControl(request);
        }).withName("Shoot Fuel");
    }

    /**
     * Commands the flywheel to shoot at a target rpm using MotionMagicVelocityVoltage
     * @param rpm The rpm to shoot at
     */
    public Command shoot(AngularVelocity rpm) {
        return shoot(() -> rpm);
    }

    /**
     * Commands the flywheel to interpolate the RPM to shoot at depending on distance to the hub
     * @param distance distance to the hub in meters
     * @return Command to shoot the flywheel at the target rpm
     */
    public Command shootInterpolatedRpm(Supplier<Distance> distance) {
        return shoot(() -> Maps.getRpmFromDistance(distance))
        .withName("Shoot at Interpolated Rpm");
    }

    public Command setHoodAngle(Supplier<Angle> target) { //? I dont think this needs to be a supplier anymore
        return runOnce(() -> {
            Angle angle = target.get();
            if (angle.gt(ShooterK.maxHoodAngle)) angle = ShooterK.maxHoodAngle;
            else if (angle.lt(ShooterK.minHoodAngle)) angle = ShooterK.minHoodAngle;

            MotionMagicVoltage request = new MotionMagicVoltage(angle);
            SmartDashboard.putNumber("target angle (degrees)", angle.in(Degrees));
            talonHood.setControl(request);
        }).withName("Set Hood Angle");
    }

    // Convenience overload for static angles — delegates up
    public Command setHoodAngle(Angle target) {
        return setHoodAngle(() -> target);
    }

    // // Interpolated — delegates up, no duplicated logic
    // public Command setInterpolatedHoodAngle(Supplier<Distance> distance) {
    //     return setHoodAngle(() -> Maps.getHoodAngleFromDistance(distance))
    //     .withName("Set Hood Interpolated Angle");
    // }


    @Logged(name = "Hood Talon Position (deg)")
    public double getHoodTalonPositionDeg() {
        return talonHood.getPosition().getValue().in(Degrees);
    }

    @Logged(name = "Hood Talon Position (rot)")
    public double getHoodTalonPositionRot() {
        return talonHood.getPosition().getValue().in(Rotations);
    }

    /**
     * Returns the given target as rotations of the hood in a 1 motor rotation : 2.05 degrees of the hood
     * @param target angle in degrees
     * @return
     */
    public Angle asRotations(Angle rot) {
        return Rotations.of(rot.in(Degrees)/2.05);
    }

    /**
     * Sets the hood to its minimum angle
     */
    // public Command stow() {
    //     // return runOnce(() -> setHoodAngle(ShooterK.minHoodAngle))
    //     .withName("Stow");
    // }

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
