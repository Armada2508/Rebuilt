package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretK;
import frc.robot.lib.util.Util;

@Logged
public class Turret extends SubsystemBase {

    private final TalonFX talon = new TalonFX(TurretK.talonId);
    /* 
     * ****************** Encoder information
     * API Documentation: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DutyCycleEncoder.html
     * Encoders (Software): https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
     * Encoders (Hardware): https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html
     */
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(TurretK.channel, TurretK.fullRange.in(Degrees), TurretK.expectedZero.in(Degrees));
    private final CANcoder canCoder = new CANcoder(0);
    public Turret() {
        configTalons();
        configMotionMagic();
        // configAbsoluteEncoder();
        configCanCoder();
        if (!absoluteEncoder.isConnected()) System.out.println("Turret Absolute Encoder not connected!");
    }
    
    /**
     * Configures the talon motor
     */
    private void configTalons() {
        Util.factoryReset(talon);
        Util.brakeMode(talon);
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
        .withMotionMagicAcceleration(TurretK.motionMagicAcceleration)
        .withMotionMagicCruiseVelocity(TurretK.motionMagicVelocity);
        talon.getConfigurator().apply(motionMagicConfig);
    }

    // private void configAbsoluteEncoder() {
    //     absoluteEncoder.setInverted(false); //! Verify this, because the dead gear and the turret gear spin in different directions, this may be needed
    //    absoluteEncoder.setAssumedFrequency(975.6); //^ Hz https://www.revrobotics.com/rev-11-1271/  
    //     //! Verify if it is a throughbore v1 or v2 when possible
    // }

    private void configCanCoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor = new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(1)
        .withMagnetOffset(0) //! FIND
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    }

    /**
     * Sets the angle of the turret
     * @param targetAngle The target angle
     */
    public void setAngle(Angle targetAngle) {
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
    @Logged(name = "Turret Angle (degrees)")
    public Angle getAngle() { //! Verify this
        double theta = Degrees.of(
                    Rotations.of(canCoder.getAbsolutePosition().getValue().in(Rotations)/*  absoluteEncoder.get() */)
                    .plus(
                        TurretK.CANCoderOffset
                    ).in(Rotations)
                ).times(
                    TurretK.encoderToTurretGearRatio
                ).in(Degrees);
        return Degrees.of(theta); //^ pls work this is annoying to math out
    }

    /**
     * Get the angular velocity of the main turret gear in rps
     * @return Angular velocity in rotations per second
     */
    public AngularVelocity getVelocity() {
        return talon.getVelocity().getValue().times(TurretK.krakenToTurretGearRatio);
    }

    /**
     * Stops the turret
     */
    public void stop() {
        talon.setControl(new NeutralOut());
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
