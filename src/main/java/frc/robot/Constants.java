package frc.robot;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import java.io.File;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static class DriveK {
        // Larger number = faster rate of change, limit is in units of (units)/second. In this case the joystick [-1, 1].
        public static final Pair<Double, Double> translationAccelLimits = Pair.of(1.25, 2.0); 
        public static final Pair<Double, Double> rotationAccelLimits = Pair.of(1.0, 2.0);
        public static final double elevatorAccelScaling = 0.5; // Acceleration is halved when elevator is at max height

        public static final double driveSpeedModifier = 0; // 0.5
        public static final double rotationSpeedModifier = 0; //0.5
        public static final double exponentialControl = 1.75;
    }
  
    public static class ControllerK {
        public static final int xboxPort = 0;
        public static final double leftJoystickDeadband = 0.15;
        public static final double rightJoystickDeadband = 0.15;

        // Teleop Alignment Overriding
        public static final double overrideThreshold = 0.14;
        public static final Time overrideTime = Seconds.of(0.25);
    }
    
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
            .withSupplyCurrentLimit(Amps.of(70)).withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Amps.of(75)).withStatorCurrentLimitEnable(true);
 
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

    public static class ShooterK { //! find motor ID and proper measurements
        public static final int talonShooterLeftID = 12;
        public static final int talonShooterRightID = 13;
        public static final int talonHoodID = 14;

        //& Motion Magic
        public static final AngularVelocity motionMagicHoodVelocity = RotationsPerSecond.of(0); //! Tune
        public static final AngularAcceleration motionMagicHoodAcceleration = RotationsPerSecondPerSecond.of(0); //! Tune

        public static final AngularAcceleration motionMagicFlywheelAcceleration = RotationsPerSecondPerSecond.of(80);

        //& Hood angle limit
        public static final Angle minHoodAngle = Degrees.of(23.35);
        public static final Angle maxHoodAngle = Degrees.of(62.8);

        //& Shooter rpm limit
        public static final AngularVelocity staticRpm = RPM.of(2900); // 2900

        //& Shooter max and min RPM
        public static final AngularVelocity flywheelVelocityUpperThreshold = RPM.of(2980);
        public static final AngularVelocity flywheelVelocityLowerThreshold = RPM.of(2860);



        //& Gear Ratios
        public static final double motorToEncoderGearRatio = 20; //these numbers should be right now
        public static final double motorToHoodGearRatio = 800/350;

        //& PID
        public static final double hoodKP = 0; //! tune
        public static final double hoodKD = 0; //! tune
        public static final double hoodKS = 0; //! tune
        public static final double hoodKV = 0; //! tune

        public static final double flywheelKP = 0.05; //! tune
        public static final double flywheelKD = 0; //! tune
        public static final double flywheelKS = 0.15; //! tune
        public static final double flywheelKV = 0.122; //! tune

        //& Current Limits
        public static final Current hoodMaxStatorCurrent = Amps.of(50); // Amps //! Tune
        //// public static final Current hoodMaxSupplyCurrent = Amps.of(0); // Amps //! Tune
        public static final Current shooterMaxStatorCurrent = Amps.of(50); // Amps //! Tune
        //// public static final Current shooterMaxSupplyCurrent = Amps.of(0); // Amps //! Tune

        //& Configs
        public static final Slot0Configs hoodPidConfig = new Slot0Configs()
        .withKP(hoodKP)
        .withKD(hoodKD)
        .withKS(hoodKS)
        .withKV(hoodKV);

        public static final Slot0Configs flywheelPidConfig = new Slot0Configs()
        .withKP(flywheelKP)
        .withKD(flywheelKD)
        // .withKS(flywheelKS)
        .withKV(flywheelKV);

        public static final SoftwareLimitSwitchConfigs hoodSoftwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxHoodAngle)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(maxHoodAngle);

        // public static final SoftwareLimitSwitchConfigs shooterSoftwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
        // .withForwardSoftLimitEnable(true)
        // .withForwardSoftLimitThreshold(minRpm.in(RPM))
        // .withReverseSoftLimitEnable(true)
        // .withReverseSoftLimitThreshold(maxRpm.in(RPM));
        
        public static final CurrentLimitsConfigs hoodCurrentLimitsConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(hoodMaxStatorCurrent);
        // .withSupplyCurrentLimit(hoodMaxSupplyCurrent);

        public static final CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(shooterMaxStatorCurrent);
        // .withSupplyCurrentLimit(shooterMaxSupplyCurrent);

        public static final FeedbackConfigs gearRatioConfig = new FeedbackConfigs()
        .withSensorToMechanismRatio(motorToHoodGearRatio);
    }
    
    public static class TurretK {
        public static final int talonId = 15; 
        //^ This may be bad, idk if can reserves id's
        
        //& Absolute Encoder
        public static final int channel = 0; //! Ask Electrical
        public static final Angle fullRange = Degrees.of(360); //! VERIFY THIS!!!!!!!
        public static final Angle expectedZero = Degrees.of(180); 
        public static final Angle absoluteEncoderOffset = Degrees.of(0); //! Find
        
        //& Gear Ratios
        public static final double krakenToTurretGearRatio = 50; //these numbers should be right now
        public static final double encoderToTurretGearRatio = 10;

        //& Motion Magic
        public static final AngularVelocity motionMagicVelocity = DegreesPerSecond.of(0); //! Find
        public static final AngularAcceleration motionMagicAcceleration = DegreesPerSecondPerSecond.of(0); //! Find

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
    
    public static class IntakeK {
        public static final int rollerID = 1;
        public static final int extenderID = 2;
    
        // Wheel current limit configs
        public static final int rollerCurrentLimit = 30; // amps //! Tune

        // Arm current limit configs
        public static final int extenderCurrentLimit = 20; // amps //! Tune

        // Soft switch limits
        public static final Distance forwardSoftLimit = Inches.of(9.25); //! Tune
        public static final Distance reverseSoftLimit = Inches.of(0);

        // Voltage limits for both the wheels and the arm
        public static final Voltage spinRollerVoltage = Volts.of(-8); //! Tune
        public static final Voltage extendVoltage = Volts.of(1); //! Tune
        public static final Voltage retractVoltage = Volts.of(-1.5);

        public static final double extenderGearRatio = 1/3.2;
        public static final Distance extenderWheelDiameter = Inches.of(1.4375);
    }   

    public static class IndexerOldK {
        public static final int talonID = 2; //! find
        
        // indexer current limit configs
        public static final SupplyCurrentLimitConfiguration indexerCurrentLimit = new SupplyCurrentLimitConfiguration(true, 0, 0, 0); //! find
       
        public static final Voltage spinindexerVoltage = Volts.of(0); //! find all values
        public static final Time jostleDuration = Seconds.of(0.25);

        /* PID configs for indexer
        public static final double kP = 0; // find all values
        public static final double kD = 0;
        public static final double kV = 0;
        public static final double kS = 0;
        

        public static final Slot0Configs coveyorPidConfig = new Slot0Configs()
        .withKP(kP)
        .withKD(kD)
        .withKV(kV)
        .withKS(kS);
        */
    }

    public static class IndexerK {
        public static final int id = 16; //! Find
        public static final Voltage indexingVoltage = Volts.of(6); //! Find
        public static final CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(0)) //! Find
        .withSupplyCurrentLimit(Amps.of(0)); //! Find
    }

        public static class HopperK {
        public static final int timeOfFlightIdTop = 0; //! find these
        public static final int timeOfFlightIdBottom = 1; //! find these
        public static final Distance hopperBottomDetectionRange = Inches.of(0);
        public static final Distance hopperTopDetectionRange = Inches.of(0);
    }

        public static class VisionK {
        public static final String frontCameraName = "LumacamFront"; // 7.5, 34.77, 5.22
        public static final String backCameraName = "ArducamSide";
        public static final Transform3d robotToFrontCamera = new Transform3d(Inches.of(1), Inches.of(-12.642), Inches.of(5.843), new Rotation3d(Degrees.of(0), Degrees.of(-16), Degrees.of(180)));
        public static final Transform3d robotToSideCamera = new Transform3d(Inches.of(0.051), Inches.of(10.983), Inches.of(11.435), new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(67.33)));
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
