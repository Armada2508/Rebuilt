package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
import frc.robot.lib.util.Util;

@Logged
public class Shooter extends SubsystemBase {

    private final TalonFX talonShooter = new TalonFX(ShooterK.talonID);
    private final TalonFX talonHood = new TalonFX(ShooterK.talonHoodID);

    //private final SparkClosedLoopController sparkMaxController = talonHood.getClosedLoopController();
    
    public Shooter() {
        configTalons();
        configMotionMagic();
        //configMaxMotion(ShooterK.cruiseVelocity, ShooterK.maxAcceleration); //!figure this out (might've figured it out)
    }

    /**
     * Configures the motors
     * talonShooter NeutralMode needs to be set as coastMode for the longevity of the motor
     * (brakeMode may cause damage in a fast moving motor)
     */
    public void configTalons() {
        Util.factoryReset(talonShooter, talonHood);
        Util.coastMode(talonShooter);
        Util.brakeMode(talonHood);

        talonShooter.getConfigurator().apply(ShooterK.shooterCurrentLimitsConfigs);
        talonShooter.getConfigurator().apply(ShooterK.shooterPidConfig);

        talonHood.getConfigurator().apply(ShooterK.hoodPidConfig);
        talonHood.getConfigurator().apply(ShooterK.hoodSoftwareLimitSwitchConfig);
        talonHood.getConfigurator().apply(ShooterK.hoodCurrentLimitsConfigs);
        talonHood.getConfigurator().apply(ShooterK.gearRatioConfig);
    }

    /**
     * Configures MotionMagic and applies it to talonHood
     */
    public void configMotionMagic() {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs()
        .withMotionMagicAcceleration(ShooterK.maxAcceleration)
        .withMotionMagicCruiseVelocity(ShooterK.cruiseVelocity);
        talonHood.getConfigurator().apply(motionMagicConfig);
    }
    
    //public void configMaxMotion(AngularVelocity velocity, AngularAcceleration acceleration) {
    //    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    //    
    //    sparkMaxConfig.closedLoop.maxMotion.cruiseVelocity(velocity.in(RotationsPerSecond));
    //    sparkMaxConfig.closedLoop.maxMotion.maxAcceleration(acceleration.in(RotationsPerSecondPerSecond));
    //    sparkmaxHood.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //}

    //public Command setShooterVoltage(Voltage voltage) {
    //    return runOnce(() -> {
    //        talonShooter.setControl(new VoltageOut(voltage.in(Volts)));
    //    }).withName("Set Shooter Voltage");
    //}
    //^ I don't think we're using voltage to control the shooter so I don't believe this is needed

    /**
     * Returns the velocity in rpm of the shooting motor
     * @return
     */
    public AngularVelocity getMotorVelocity() {
        return talonShooter.getVelocity().getValue().div(60);
    }

    /**
     * Sets the shooter to a set RPM
     * @param rpm The RPM to shoot at
     * @return
     */
    public Command setShooterVelocity(AngularVelocity rpm) {
        return runOnce(() -> talonShooter.setControl(new VelocityVoltage(rpm)));
    }

    /**
     * Shoots the fuel at a static RPM
     * @return
     */
    public Command shootFuel() {
        return setShooterVelocity(ShooterK.staticRpm);
    }

    /**
     * Sets the hood to a target angle using Motion Magic
     * @param targetAngle Angle to set the hood to
     * @return runnable containing a command to command the talon
     */
    public Command setHoodAngle(Angle targetAngle) {
        //return runOnce(() -> sparkMaxController.setSetpoint(targetAngle.in(Degrees), ControlType.kMAXMotionPositionControl));
        MotionMagicVoltage request = new MotionMagicVoltage(targetAngle);
        return runOnce(() -> talonHood.setControl(request));
    }

    /**
     * Sets the hood to its minimum angle
     */
    public void zeroHood() {
        talonHood.setPosition(ShooterK.minHoodAngle);
    }

    /**
     * Stops the shooter and the hood motors from moving
     */
    public void stop() {
        talonShooter.setControl(new NeutralOut());
        talonHood.setControl(new NeutralOut());
    }
}
