package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
import frc.robot.lib.util.Util;

@Logged
public class Shooter extends SubsystemBase {
    
    private final TalonFX talonShooter = new TalonFX(ShooterK.talonID);
    public Shooter() {
        configTalons();
    }

    /**
     * The talons NeutralMode need to be set as coastMode for the longevity of the motor
     * (brakeMode may cause damage)
     */
    public void configTalons() {
        Util.factoryReset(talonShooter);
        Util.coastMode(talonShooter);
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

    public void shooterPeriodic() {
        AngularVelocity velocity = getMotorVelocity();
    }
    
    

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
    public void stop() {
        talonShooter.setControl(new NeutralOut());
    }

    public Command flywheelUpToSpeed() {
        return setShooterVelocity(getMotorVelocity())
        .andThen(Commands.waitSeconds(ShooterK.flywheelSpeedUpTime));
    }

    /*
     * Power the motors.
     * 
     * 
     * 
     * 
     * 
     */
    //!unfinished, figure it out soon
    public Command shootFuel() {
        return setShooterVoltage(ShooterK.fuelShootVoltage)
        .andThen();
    }
}
