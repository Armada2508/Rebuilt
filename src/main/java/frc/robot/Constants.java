package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.io.File;
import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {
    public static class SwerveK {
        public static final Distance driveBaseRadius = Inches.of(15.37957);
        public static final Distance driveBaseLength = Inches.of(27); // Base is a square so this is the same as the width
        public static final Time coastDisableTime = Seconds.of(10);

        // Currently Unused
        // public static final double steerGearRatio = 41.25; 
        // public static final double driveGearRatio = 4.4;

        public static final LinearVelocity maxPossibleRobotSpeed = MetersPerSecond.of(5.426);
        public static final AngularVelocity maxAngularVelocity = RadiansPerSecond.of(10.477);
        public static final CurrentLimitsConfigs driveCurrentLimitsConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(70)).withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(75)).withStatorCurrentLimitEnable(true);

        public static final CurrentLimitsConfigs steerCurrentLimitsConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Amps.of(70)).withSupplyCurrentLimitEnable(true) //! Find / Tune
            .withStatorCurrentLimit(Amps.of(75)).withStatorCurrentLimitEnable(true); //! Find / Tune
 
        // Path Constraints
        public static final LinearVelocity maxRobotVelocity = FeetPerSecond.of(6); // Should be just under 3/4 of our max possible speed, arbitrary value
        public static final LinearAcceleration maxRobotAcceleration = FeetPerSecondPerSecond.of(3.5); 
        public static final AngularVelocity maxRobotAngularVelocity = DegreesPerSecond.of(180); 
        public static final AngularAcceleration maxRobotAngularAcceleration = DegreesPerSecondPerSecond.of(270); 

        // Drive Feedforward
        public static final double kS = 0.23118;
        public static final double kV = 2.1701;
        public static final double kA = 0.15136;

        // PathPlanner
        public static final PIDConstants ppTranslationConstants = new PIDConstants(5.25, 0, 0); // m/s / m
        public static final PIDConstants ppRotationConstants = new PIDConstants(5, 0, 0); // rad/s / rad
        public static RobotConfig robotConfig; static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
        }

        // PID Alignment
        public static final PIDConstants translationConstants = new PIDConstants(5.25, 0, 0); // m/s / m of error
        public static final PIDConstants rotationConstants = new PIDConstants(5, 0, 0); // rad/s / rad of error
        public static final TrapezoidProfile.Constraints defaultTranslationConstraints = 
            new TrapezoidProfile.Constraints(Units.feetToMeters(5), Units.feetToMeters(10)); // m/s & m/s^2
        public static final TrapezoidProfile.Constraints defaultRotationConstraints = 
            new TrapezoidProfile.Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360)); // rad/s & rad/s^2
        public static final TrapezoidProfile.Constraints climbTranslationConstraints = 
            new TrapezoidProfile.Constraints(Units.feetToMeters(2), Units.feetToMeters(4)); // m/s & m/s^2
        public static final TrapezoidProfile.Constraints climbRotationConstraints = 
            new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(180)); // rad/s & rad/s^2
        public static final Distance maximumTranslationError = Inches.of(0.25);
        public static final Angle maximumRotationError = Degrees.of(0.5);

        public static final File swerveDirectory = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/swerve");
    }

public static class ControllerK {
        public static final int xboxPort = 0;
        public static final double leftJoystickDeadband = 0.15;
        public static final double rightJoystickDeadband = 0.15;

        // Teleop Alignment Overriding
        public static final double overrideThreshold = 0.14;
        public static final Time overrideTime = Seconds.of(0.25);
    }
    public static class DriveK {
        // Larger number = faster rate of change, limit is in units of (units)/second. In this case the joystick [-1, 1].
        public static final Pair<Double, Double> translationAccelLimits = Pair.of(1.25, 2.0); 
        public static final Pair<Double, Double> rotationAccelLimits = Pair.of(1.0, 2.0);
        public static final double elevatorAccelScaling = 0.5; // Acceleration is halved when elevator is at max height

        public static final double driveSpeedModifier = 0.167;
        public static final double rotationSpeedModifier = 0.167;
        public static final double exponentialControl = 1.75;
    }
  
    public static class VisionK {
        public static final String frontCameraName = "LumacamFront"; // 7.5, 34.77, 5.22
        // public static final String backCameraName = "ArducamBack";
        public static final Transform3d robotToFrontCamera = new Transform3d(Inches.of(1), Inches.of(12.642), Inches.of(5.843), new Rotation3d(Degrees.of(0), Degrees.of(-16), Degrees.of(0)));
        //                                                                      
        // public static final Transform3d robotToBackCamera = new Transform3d(Inches.of(-3.148), Inches.of(7.729), Inches.of(32.452), new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(-155)));
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

    public static class HopperK {
        public static final int timeOfFlightIdTop = 0; //! find these
        public static final int timeOfFlightIdBottom = 1; //! find these
        public static final Distance hopperBottomDetectionRange = Inches.of(0);
        public static final Distance hopperTopDetectionRange = Inches.of(0);
    }
  }
}