package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.Volt;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
import frc.robot.lib.util.Util;

@Logged
public class Shooter extends SubsystemBase {

    private final TalonFX talonShooter = new TalonFX(ShooterK.talonID);
    private final SparkMax sparkmaxHood = new SparkMax(ShooterK.sparkmaxHoodID, MotorType.kBrushless);

    private final SparkClosedLoopController sparkMaxController = sparkmaxHood.getClosedLoopController();
    
    public Shooter() {
        configTalons();
        configMaxMotion(ShooterK.cruiseVelocity, ShooterK.maxAcceleration); //!figure this out (might've figured it out)
    }

    /**
     * The talons NeutralMode need to be set as coastMode for the longevity of the motor
     * (brakeMode may cause damage)
     */
    public void configTalons() {
        Util.factoryReset(talonShooter);
        Util.coastMode(talonShooter);
    }

    public void configMaxMotion(AngularVelocity velocity, AngularAcceleration acceleration) {
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        
        sparkMaxConfig.closedLoop.maxMotion.cruiseVelocity(velocity.in(RotationsPerSecond));
        sparkMaxConfig.closedLoop.maxMotion.maxAcceleration(acceleration.in(RotationsPerSecondPerSecond));
        sparkmaxHood.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

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

    public Command hoodAngle(Angle targetAngle) {
        return runOnce(() -> sparkMaxController.setSetpoint(targetAngle.in(Degrees), ControlType.kMAXMotionPositionControl));
    }

    public void stop() {
        talonShooter.setControl(new NeutralOut());
    }
}
