package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.IntakeK;

public class Intake {

    private final TalonFX talonWheels = new TalonFX(IntakeK.talonWheelsID);
    private final TalonFX talonArm = new TalonFX(IntakeK.talonArmID);

    public Intake() {
        configTalons();
        configMotionMagic();
        
    }

    public void configTalons() {
        TalonFXConfiguration wheelsConfig = new TalonFXConfiguration();
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        
        talonWheels.getConfigurator().apply(new TalonFXConfiguration()); // factory reset
        talonWheels.getConfigurator().apply(IntakeK.pidConfig);
        talonWheels.getConfigurator().apply(IntakeK.softwareLimitConfig);
        talonArm.getConfigurator().apply(new TalonFXConfiguration());
    }
    public void configMotionMagic() {
        
    }

    public void stow() { // Put the intake up and stop the motors

    }

    public void intakeDepot() { 

    }

    public void setAngle() { // Set the angle to which the intake will move to
        
    }

    public void getAngle() { //! change return type?

    }

    public void stop() { // stop everything
        talonArm.stopMotor();
        talonWheels.stopMotor();
    }
   
}
