package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static class IntakeK {
        public static final int motorID = 0;
        

        // PID & Feedforward gains
        public static final double kP = 0; //! find all values
        public static final double kD = 0;
        public static final double kV = 0;

        // Limits
        public static final Angle maxAngle = Degrees.of(0); //! find values
        public static final Angle minAngle = Degrees.of(0);


        public static final Slot0Configs pidConfig = new Slot0Configs()
        .withKP(kP)
        .withKD(kD)
        .withKV(kV);

        public static final SoftwareLimitSwitchConfigs softwareLimitConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxAngle.in(Degrees))
        .withReverseSoftLimitThreshold(maxAngle.in(Degrees));

    }
}