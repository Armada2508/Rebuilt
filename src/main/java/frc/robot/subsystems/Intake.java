package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.Constants.IntakeK;
import frc.robot.lib.util.Util;

public class Intake {

    private final TalonFX wheels = new TalonFX(IntakeK.wheelsID);
    private final TalonFX arm = new TalonFX(IntakeK.armID);

    public Intake() {
        configTalons();
        configMotionMagic(IntakeK.maxVelocity, IntakeK.maxAcceleration);
        setAngle(IntakeK.intakeAngle);
    }

    public void configTalons() {
        Util.factoryReset(wheels, arm); // factory reset
        wheels.getConfigurator().apply(IntakeK.pidConfig);
        wheels.getConfigurator().apply(IntakeK.softwareLimitConfig);

        arm.getConfigurator().apply(IntakeK.pidConfig);
        arm.getConfigurator().apply(IntakeK.softwareLimitConfig);
    }

    public void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(DegreesPerSecond);
        motionMagicConfigs.MotionMagicAcceleration = acceleration.in(DegreesPerSecondPerSecond);
        
        arm.getConfigurator().apply(motionMagicConfigs);
    }

    public void stow() { // Put the intake up and stop the motors
       setAngle(IntakeK.stowAngle);
    }

    public void intakeDepot() { 
        setAngle(IntakeK.intakeDepotAngle);
    }

    public void setAngle(Angle angle) { // Set the angle to which the intake will move to
        MotionMagicVoltage request = new MotionMagicVoltage(angle);
        arm.setControl(request);
    }

    public Angle getAngle() {
    return arm.getPosition();
    }

    public void spinWheels() {
        wheels.setControl(new VoltageOut(IntakeK.spinWheelsVoltage)); //! find value
    }

    public void stop() { // stop everything
        wheels.setControl(new NeutralOut());
        arm.setControl(new NeutralOut());
    }
   
}
