package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretK;
import frc.robot.lib.util.Util;

public class Turret extends SubsystemBase {

    private final TalonFX talon = new TalonFX(TurretK.talonId);

    public Turret() {
        configTalons();
        configMotionMagic();
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
     * @param targetAngle The target angle
     * @return 
     */
    public Command setAngleCommand(Angle targetAngle) {
        return runOnce(() -> setAngle(targetAngle)); //! Check

    }

    public Angle getAngle() {
        return talon.getPosition().getValue();
    }

}
