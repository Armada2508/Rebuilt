package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretK;
import frc.robot.lib.util.Util;

public class Turret extends SubsystemBase {

    private final TalonFX talon = new TalonFX(TurretK.talonId);
    /* 
     * ****************** Encoder information
     * API Documentation: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DutyCycleEncoder.html
     * Encoders (Software): https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
     * Encoders (Hardware): https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html
     */
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(TurretK.channel, TurretK.fullRange.in(Degrees), TurretK.expectedZero.in(Degrees));


    public Turret() {
        configTalons();
        configMotionMagic();
        configAbsoluteEncoder();
        if (!absoluteEncoder.isConnected()) System.out.println("Turret Absolute Encoder not connected!");
    }
    
    /**
     * Configures the talon motor
     */
    private void configTalons() {
        Util.factoryReset(talon);
        Util.brakeMode(talon); // We likely want to use brake for the turret, double check though
        talon.getConfigurator().apply(TurretK.pidConfig);
        talon.getConfigurator().apply(TurretK.softwareLimitSwitchConfig);
        talon.getConfigurator().apply(TurretK.currentLimitConfig);
        talon.getConfigurator().apply(TurretK.gearRatioConfig);
        // no hard limit switch likely
        talon.setPosition(TurretK.defaultPosition); // zero the turret
    }

    /**
     * Configures the MotionMagic
     */
    private void configMotionMagic() {
        MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs()
        .withMotionMagicAcceleration(TurretK.maxAcceleration)
        .withMotionMagicCruiseVelocity(TurretK.maxVelocity);
        talon.getConfigurator().apply(motionMagicConfig);

    }

    private void configAbsoluteEncoder() {
        absoluteEncoder.setInverted(false); //! Verify this, because the dead gear and the turret gear spin in different directions, this may be needed
        absoluteEncoder.setAssumedFrequency(0); //^ 1000 Hz if we use the REV Throughbore, 244 Hz if we use the CTRE Mag Encoder
    }

    @Override
    public void periodic() {
    }

    /**
     * Sets the angle of the turret
     * @param targetAngle The target angle
     */
    public void setAngle(Angle targetAngle) {
        // talon.setControl(yawControl.withPosition(translateYaw(targetAngle)));
        MotionMagicVoltage request = new MotionMagicVoltage(targetAngle.in(Rotations)); //^ Verify if we need to do the .in()
        talon.setControl(request);

    }

    /**
     * Constructs a command to set the angle of the turret
     * ? We should know whether or not we measure from 0-360 or -180 to 180. The latter seems safer as with the former, the "zero" would be at 180 degrees.
     * @param targetAngle The target angle
     * @return Command to set the angle
     */
    public Command setAngleCommand(Angle targetAngle) {
        return runOnce(() -> setAngle(targetAngle)); //! Check

    }

    /**
     * Returns the angle of the turret as read by the absolute encoder
     * @return
     */
    public Angle getAngle() { //! Verify this
        return Degrees.of((Rotations.of(absoluteEncoder.get()).in(Degrees) + TurretK.absoluteEncoderOffset.in(Degrees)) /* % Constants.degreesPerRotation */ ); //* Re-go over the and stuff for this.
        // return Degrees.of(realEncoderReading * TurretK.encoderToTurretGearRatio); //? Can we use the Degrees.of() here?
    }

    /**
     * Stops the turret
     */
    public void stop() {
        talon.setControl(new NeutralOut());
    }

}
