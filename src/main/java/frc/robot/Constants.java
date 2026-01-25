package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.Matrix;


public class Constants {
    public static class IntakeK {
        public static final int wheelsID = 0; //! find, may change
        public static final int extensionID = 1;
        
        // Limits
        public static final Distance maxDistance = Inches.of(0); //! find values

        // Wheel current limit configs
        public static final CurrentLimitsConfigs wheelCurrentConfigs = new CurrentLimitsConfigs() //! find values
        .withStatorCurrentLimit(Amps.of(0))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(0))
        .withSupplyCurrentLowerLimit(Amps.of(0))
        .withSupplyCurrentLimitEnable(true);

        // Arm current limit configs
        public static final CurrentLimitsConfigs talonCurrentConfigs = new CurrentLimitsConfigs() //! find values
        .withStatorCurrentLimit(Amps.of(0))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Amps.of(0))
        .withSupplyCurrentLowerLimit(Amps.of(0))
        .withSupplyCurrentLimitEnable(true);

        public static final Voltage spinWheelsVoltage = Volts.of(0); //! find values
        public static final Voltage extendVoltage = Volts.of(0);

        public static final LinearVelocity maxVelocity = MetersPerSecond.of(0); //! find
        public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(0);

        // Software limit switch configs for arm
        public static final SoftwareLimitSwitchConfigs softwareLimitConfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxDistance.in(Inches))
        .withReverseSoftLimitThreshold(maxDistance.in(Inches));

        public static final Distance wheelDiameter = Inches.of(1.32); //! double check?

        public static final Voltage extendVolts = Volts.of(0); //! find
    }

        


    public static class VisionK {
        public static final String frontCameraName = "ArducamFront"; // 7.5, 34.77, 5.22
        public static final String backCameraName = "ArducamBack"; 
        public static final Transform3d robotToFrontCamera = new Transform3d(Inches.of(0.577), Inches.of(-1.023), Inches.of(29.223), new Rotation3d(Degrees.of(11.5), Degrees.of(30.75), Degrees.of(5.8)));
        public static final Transform3d robotToBackCamera = new Transform3d(Inches.of(-3.148), Inches.of(7.729), Inches.of(32.452), new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(-155)));
        // Acceptable height of pose estimation to consider it a valid pose
        public static final Distance maxPoseZ = Inches.of(12);
        public static final Distance minPoseZ = Inches.of(-6);
        // Used in scaling the standard deviations by average distance to april tags
        public static final Distance baseLineAverageTagDistance = Inches.of(84);
        public static final Distance maxAverageTagDistance = Inches.of(160);
        // Vision Standard Deviations (Meters, Meters, Radians)
        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(Units.feetToMeters(3), Units.feetToMeters(3), Units.degreesToRadians(360));
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(Units.feetToMeters(1.5), Units.feetToMeters(1.5), Units.degreesToRadians(180));
        public static final Matrix<N3, N1> untrustedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
}