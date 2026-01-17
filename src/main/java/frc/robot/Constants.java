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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.LinearAccelerationUnit;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.units.measure.AngularAcceleration;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.units.measure.LinearAcceleration;
// import edu.wpi.first.units.measure.LinearVelocity;
// import edu.wpi.first.units.measure.Time;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.Filesystem;
// import frc.robot.lib.util.Encoder;


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