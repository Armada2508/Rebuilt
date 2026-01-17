package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;


public class Constants {
    public static class IntakeK {
        public static final int armID = 0;
        public static final int wheelsID = 1;
        TalonFXConfiguration wheelsConfig = new TalonFXConfiguration();
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        
        // PID & Feedforward gains
        
        public static final double kP = 0; //! find all values
        public static final double kD = 0;
        public static final double kV = 0;
        public static final double kG = 0;

        // Limits
        public static final Angle maxAngle = Degrees.of(0); //! find values
        public static final Angle minAngle = Degrees.of(0);
        public static final Angle stowAngle = Degrees.of(0);
        public static final Angle intakeDepotAngle = Degrees.of(0);
        public static final Angle intakeAngle = Degrees.of(0);
        public static final Voltage spinWheelsVoltage = Volts.of(0);
        public static final Voltage stowVoltage = Volts.of(0);

        public static final Slot0Configs pidConfig = new Slot0Configs()
        .withKP(kP)
        .withKD(kD)
        .withKV(kV)
        .withKG(kG);

        public static final AngularVelocity maxVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(0);
        // Motion Magic settings
        

        public static final SoftwareLimitSwitchConfigs softwareLimitConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxAngle.in(Degrees))
        .withReverseSoftLimitThreshold(maxAngle.in(Degrees));

        

        


    }
}