package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.RotationsPerSecond;
// import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.Volt;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
// import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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
     * The talons NeutralMode need to be set as coastMode for the longevity of the motor
     * (brakeMode may cause damage)
     */
    public void configTalons() {
        Util.factoryReset(talonShooter, talonHood);
        Util.coastMode(talonShooter, talonHood);  //!Find out if talonHood needs to be set in coastmode
    }

    public void configMotionMagic() {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs()
        .withMotionMagicAcceleration(ShooterK.maxAcceleration)
        .withMotionMagicCruiseVelocity(ShooterK.cruiseVelocity);
    }
    
    //public void configMaxMotion(AngularVelocity velocity, AngularAcceleration acceleration) {
    //    SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    //    
    //    sparkMaxConfig.closedLoop.maxMotion.cruiseVelocity(velocity.in(RotationsPerSecond));
    //    sparkMaxConfig.closedLoop.maxMotion.maxAcceleration(acceleration.in(RotationsPerSecondPerSecond));
    //    sparkmaxHood.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //}

    public Command setShooterVoltage(Voltage voltage) {
        return runOnce(() -> {
            talonShooter.setControl(new VoltageOut(voltage.in(Volts)));
        }).withName("Set Shooter Voltage");
    }
    /**
     * Function that returns the Angular Velocity of talonShooter
     *
     *
     * @return
     */
    public AngularVelocity getMotorVelocity() {
        return talonShooter.getVelocity().getValue().div(60);
    }

    public Command setShooterVelocity(AngularVelocity rpm) {
        return runOnce(() -> talonShooter.setControl(new VelocityVoltage(rpm)));
    }

    public Command shootFuel(AngularVelocity velocity) {
        return setShooterVelocity(velocity);
    }

    //! double check if this is right
    public Command setHoodAngle(Angle targetAngle) {
        //return runOnce(() -> sparkMaxController.setSetpoint(targetAngle.in(Degrees), ControlType.kMAXMotionPositionControl));
        return runOnce(() -> talonHood.setPosition(targetAngle.in(Degrees)));
    }

    public void stop() {
        talonShooter.setControl(new NeutralOut());
    }
}
