package frc.robot.subsystems.shooting;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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

    private final TalonFX talonShooterLeft = new TalonFX(ShooterK.talonID); // As viewed from the back of the turret structure
    private final TalonFX talonShooterRight = new TalonFX(ShooterK.talonFollowID); // As viewed from the back of the turret structure
    private final TalonFX talonHood = new TalonFX(ShooterK.talonHoodID);

    
    
    public Shooter() {
        configTalons();
        configMotionMagic();
    }

    /**
     * Configures the motors
     * talonShooterLeft NeutralMode needs to be set as coastMode for the longevity of the motor
     * (brakeMode may cause damage in a fast moving motor)
     */
    public void configTalons() {
        Util.factoryReset(talonShooterLeft, talonShooterRight, talonHood);
        Util.coastMode(talonShooterLeft, talonShooterRight);
        Util.brakeMode(talonHood);

        talonShooterRight.setControl(new StrictFollower(talonShooterLeft.getDeviceID()));

        MotorOutputConfigs invertConfig = new MotorOutputConfigs();
        invertConfig.Inverted = InvertedValue.Clockwise_Positive;

        talonShooterLeft.getConfigurator().apply(invertConfig); //! Verify that this is correct
        talonShooterLeft.getConfigurator().apply(ShooterK.shooterCurrentLimitsConfigs);
        talonShooterLeft.getConfigurator().apply(ShooterK.shooterPidConfig);

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
        .withMotionMagicAcceleration(ShooterK.motionMagicAcceleration)
        .withMotionMagicCruiseVelocity(ShooterK.motionMagicVelocity);
        talonHood.getConfigurator().apply(motionMagicConfig);
    }

    //public Command setShooterVoltage(Voltage voltage) {
    //    return runOnce(() -> {
    //        talonShooterLeft.setControl(new VoltageOut(voltage.in(Volts)));
    //    }).withName("Set Shooter Voltage");
    //}
    //^ I don't think we're using voltage to control the shooter so I don't believe this is needed

    /**
     * Returns the velocity in rpm of the shooting motor
     * @return
     */
    public AngularVelocity getMotorVelocity() {
        return talonShooterLeft.getVelocity().getValue().div(60);
    }

    /**
     * Sets the shooter to a set RPM
     * @param rpm The RPM to shoot at
     * @return
     */
    public Command setShooterVelocity(AngularVelocity rpm) {
        return runOnce(() -> talonShooterLeft.setControl(new VelocityVoltage(rpm)));
    }

    /**
     * Shoots the fuel at a static RPM
     * @return
     */
    public Command shootFuel() {
        // return setShooterVelocity(ShooterK.staticRpm);
        return runOnce(() -> talonShooterLeft.setControl(new VoltageOut(ShooterK.shooterVoltage)));
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
    public Command stow() {
        return runOnce(() -> talonHood.setPosition(ShooterK.minHoodAngle));
    }

    /**
     * Stops the shooter and the hood motors from moving
     */
    public Command stop() {
        return runOnce(() -> talonShooterLeft.setControl(new NeutralOut()))
        .andThen(runOnce(() -> talonHood.setControl(new NeutralOut())));
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
