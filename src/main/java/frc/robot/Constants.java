package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.Matrix;
public class Constants {
    public static double degreesPerRotation = 360;
    
    public static class ShooterK { //! find motor ID and proper measurements
        public static final int talonID = 0;
        public static final int talonHoodID = 1;

        public static final Voltage fuelShootVoltage = Volts.of(1);
        public static final Time flywheelSpeedUpTime = Seconds.of(0.5);
        public static final AngularVelocity cruiseVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(0);

        public static final Angle minHoodAngle = Degrees.of(62.8); //! double check that these seem ok before testing?
        public static final Angle maxHoodAngle = Degrees.of(102.3);

        public static final AngularVelocity minRpm = RPM.of(0);
        public static final AngularVelocity staticRpm = RPM.of(0); //! Final
        public static final AngularVelocity maxRpm = RPM.of(0); //! Find
        
    }
    public static class TurretK {
        public static final int talonId = 0; //! Find
        
        //& Absolute Encoder
        public static final int channel = 0; //! Ask Electrical
        public static final Angle fullRange = Degrees.of(360); //! VERIFY THIS!!!!!!!
        public static final Angle expectedZero = Degrees.of(180); //! VERIFY THIS!!!!!
        public static final Angle absoluteEncoderOffset = Degrees.of(0); //! Find
        
        //& Gear Ratios
        public static final double krakenToTurretGearRatio = 0; //! Ask mechanical
        public static final double encoderToTurretGearRatio = 0; //! Ask Mechanical

        //& Motion Magic
        public static final AngularVelocity maxVelocity = DegreesPerSecond.of(0); //! Find
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(0); //! Find

        //& Angles
        public static final Angle defaultPosition = Degrees.of(0); //^ Turret MUST be facing towards the exact front of the robot on startup. This is ESSENTIAL to zeroing. This is 
                                                                  //^ This is likely outdated with us using an absolute encoder now. Up to testing & Debugging

        // We dont go the maximum rotation to avoid wrap-around error and risk confusing the Absolute encoder, might not be needed though.
        public static final Angle maxAngle = Degrees.of(179.5);  //! Verify / Check
        public static final Angle minAngle = Degrees.of(-179.5); //! Verify / Check

        //& Currents
        public static final Current maxStatorCurrent = Amps.of(0); //! Find / Verify
        public static final Current maxSupplyCurrent = Amps.of(0); //! Find / Verify

        //& PID
        public static final double kP = 0; //! Tune
        public static final double kD = 0; //! Tune
        public static final double kS = 0; //! Tune
        public static final double kV = 0; //! Tune

        //& Configs
        public static final Slot0Configs pidConfig = new Slot0Configs()
        .withKP(kP)
        .withKD(kD)
        .withKS(kS)
        .withKV(kV);

        public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxAngle)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(minAngle);

        public static final CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(maxStatorCurrent)
        .withSupplyCurrentLimit(maxSupplyCurrent);

        public static final FeedbackConfigs gearRatioConfig = new FeedbackConfigs()
        .withSensorToMechanismRatio(krakenToTurretGearRatio);
    }

    public static class SuperstructureK {
        //? Might not be needed?
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
