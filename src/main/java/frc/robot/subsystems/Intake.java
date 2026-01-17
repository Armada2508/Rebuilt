package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeK;
import frc.robot.lib.util.Util;

@Logged
public class Intake extends SubsystemBase{

    private final TalonFX wheels = new TalonFX(IntakeK.wheelsID);
    private final TalonFX arm = new TalonFX(IntakeK.armID);

    public Intake() {
        configTalons();
        configMotionMagic(IntakeK.maxVelocity, IntakeK.maxAcceleration);
    }
    /**
     * factory resets motors and applies PID, software limits, and current limits
     */
    private void configTalons() {
        Util.factoryReset(wheels, arm);

        wheels.getConfigurator().apply(IntakeK.wheelPidConfig);
        wheels.getConfigurator().apply(IntakeK.softwareLimitConfig);
        wheels.getConfigurator().apply(IntakeK.wheelCurrentConfigs);

        arm.getConfigurator().apply(IntakeK.armPidConfig);
        arm.getConfigurator().apply(IntakeK.softwareLimitConfig);
        arm.getConfigurator().apply(IntakeK.armCurrentConfigs);
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
     * Sets angles
     */
    public Command setAngleCommand(Angle targetAngle) { 
       return runOnce(() -> {
        setAngle(targetAngle);
       })
       .withName("Set Angle");
    }
    /**
     * Sets the angle specified for the arm
     * @param angle
     */
    public void setAngle(Angle angle) { // Set the angle to which the intake will move to
        MotionMagicVoltage request = new MotionMagicVoltage(angle);
        arm.setControl(request);
    }
    /**
     * Gets angle of intake
     * @return
     */
    public Angle getAngle() {
        return arm.getPosition().getValue();
    }
    /**
     * Spins intake wheels/motors
     */
    public void spinWheels() { 
        wheels.setControl(new VoltageOut(IntakeK.spinWheelsVoltage)); //! find value
    }

    /**
     * Stops motors
     */
    public void stop() { 
        wheels.setControl(new NeutralOut());
        arm.setControl(new NeutralOut());
    }
   
}
