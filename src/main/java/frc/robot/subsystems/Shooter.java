package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
import frc.robot.lib.util.Util;

@Logged
public class Shooter extends SubsystemBase {
    
    // AngularVelocity velocity;

    // AngularVelocity locativeVelocity;

    private final TalonFX talonShooter = new TalonFX(ShooterK.talonID);
    private final SparkMax sparkmaxHood = new SparkMax(ShooterK.sparkmaxHoodID, MotorType.kBrushless);

    public Shooter() {
        configTalons();
        configMotionMagic(ShooterK.maxVelocity, ShooterK.maxAcceleration); //!figure this out
    }

    /**
     * The talons NeutralMode need to be set as coastMode for the longevity of the motor
     * (brakeMode may cause damage)
     */
    public void configTalons() {
        Util.factoryReset(talonShooter);
        Util.coastMode(talonShooter);
    }

    public void configMotionMagic() {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.withMotionMagicAcceleration(null)
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


    public void stop() {
        talonShooter.setControl(new NeutralOut());
    }

    public Command hoodAngle(Angle targetAngle) {
        return runOnce(() -> sparkmaxHood.)
    }
    // public void shooterPeriodic() {
    //     AngularVelocity velocity = getMotorVelocity();
    // }
    
    //public Command brakeShooter(){
    //    return runOnce(() -> {
    //        var request = new StaticBrake();
    //        talonShooter.setControl(request);
    //    }).withName("Brake Shooter");
    //}

    /**
     * Sets shooter motor to its NeutralMode
     * (in this case coastMode)
     */
    
    /**
     * Flywheels on the shooter need time to get up to speed for an effective shot
     * The velocity needed will vary depending on the distance of the robot to the hub
     * flywheelUpToSpeed has the motors speed up to the intended velocity before fuel leaves the turret for the shooter
     * 
     * 
     */
    // public Command flywheelUpToSpeed() {
    //     return setShooterVelocity(locativeVelocity)
    //     .andThen(Commands.waitSeconds(ShooterK.flywheelSpeedUpTime.in(Seconds)));
    // }

    /*
     * Power up the motors
     * Calculate the velocity required based on where you are on the field.
     * Shoot the fuel...?????????????????????
     * 
     * 
     * 
     */
    
}
