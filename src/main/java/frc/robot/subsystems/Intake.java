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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;
import frc.robot.lib.util.Util;

public class Intake extends SubsystemBase implements Loggable {

    private final TalonFX wheels = new TalonFX(IntakeK.wheelsID);
    private final TalonFX arm = new TalonFX(IntakeK.armID);

    public Intake() {
        configTalons();
        configMotionMagic(IntakeK.maxVelocity, IntakeK.maxAcceleration);
    }
    /**
     * factory resets motors and applies PID + software limits
     * javadoc
     */
    private void configTalons() {
        Util.factoryReset(wheels, arm); // factory reset
        wheels.getConfigurator().apply(IntakeK.wheelPidConfig);
        wheels.getConfigurator().apply(IntakeK.softwareLimitConfig);

        arm.getConfigurator().apply(IntakeK.armPidConfig);
        arm.getConfigurator().apply(IntakeK.softwareLimitConfig);
    }
    /**
     * configs Motion Magic
     * @param velocity
     * @param acceleration
     */
    private void configMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = velocity.in(DegreesPerSecond);
        motionMagicConfigs.MotionMagicAcceleration = acceleration.in(DegreesPerSecondPerSecond);
        
        arm.getConfigurator().apply(motionMagicConfigs);
    }
    /**
     * javadoc
     */
    public Command setAngleCommand(Angle targetAngle) { // Put the intake up and stop the motors
       return runOnce(() -> {
        setAngle(targetAngle);
       })
       .withName("Set Angle");
    }
    /**
     * javadoc
     */
    public void intakeDepot() { 
        setAngle(IntakeK.intakeDepotAngle);
    }
    /**
     * Sets the angle specified
     * @param angle
     */
    public void setAngle(Angle angle) { // Set the angle to which the intake will move to
        MotionMagicVoltage request = new MotionMagicVoltage(angle);
        arm.setControl(request);
    }
    /**
     * javadoc
     * @return
     */
    public Angle getAngle() {
        return arm.getPosition().getValue();
    }
    /**
     * javadoc
     */
    public void spinWheels() { // spins the intake wheels/motor
        wheels.setControl(new VoltageOut(IntakeK.spinWheelsVoltage)); //! find value
    }

    /**
     * javadoc
     */
    public void stop() { // stop everything
        wheels.setControl(new NeutralOut());
        arm.setControl(new NeutralOut());
    }
   
}
