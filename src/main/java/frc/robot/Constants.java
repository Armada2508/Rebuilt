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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

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

        public static final double driveSpeedModifier = 0.25; // 0.5
        public static final double rotationSpeedModifier = 0.25; //0.5
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
        public static final int CANCoderID = 0;

        //& Motion Magic
        public static final AngularVelocity motionMagicHoodVelocity = RotationsPerSecond.of(0.5); //! Tune
        public static final AngularAcceleration motionMagicHoodAcceleration = RotationsPerSecondPerSecond.of(0.5); //! Tune

        public static final AngularAcceleration motionMagicFlywheelAcceleration = RotationsPerSecondPerSecond.of(150);

        //& Hood angle limit
        public static final Angle minHoodAngle = Degrees.of(0);
        public static final Angle maxHoodAngle = Degrees.of(40);

        //& Static Set Points
        public static final AngularVelocity staticPassingRpm = RPM.of(3000); //! Tune
        public static final AngularVelocity staticStealingRpm = RPM.of(3000); //! Tune

        public static final Angle staticPassingHoodAngle = Degrees.of(30); //! Tune
        public static final Angle staticStealingHoodAngle = Degrees.of(35); //! Tune

        //& Gear Ratios
        public static final double motorToEncoderGearRatio = 20.0 / 1.0; //these numbers should be right now
        public static final double motorToHoodGearRatio = 40.0 / 350.0;
        public static final double encoderToHoodGearRatio = 0.114;
        public static final double motorToHoodDegreeGearRatio = 2.05; // 1 Rotation of the motor shaft = 2.05 Degrees of the hood.
        

        //& PID
        public static final double hoodKP = 195; //! tune 
        public static final double hoodKD = 0; //! tune
        public static final double hoodKS = 0.2; //! tune 0.2
        public static final double hoodKG = 0.05;
        public static final double hoodKV = 1; //! tune

        public static final double flywheelKP = 0.8875;
        public static final double flywheelKD = 0; 
        public static final double flywheelKS = 0.15; 
        public static final double flywheelKV = 0.122;

        //& Current Limits
        public static final Current hoodMaxStatorCurrent = Amps.of(50); // Amps //! Tune
        public static final Current shooterMaxStatorCurrent = Amps.of(50); // Amps //! Tune

        //& Configs
        public static final Slot0Configs hoodPidConfig = new Slot0Configs()
        .withKP(hoodKP)
        .withKD(hoodKD)
        .withKG(hoodKG)
        .withKS(hoodKS);
        // .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        public static final Slot0Configs flywheelPidConfig = new Slot0Configs()
        .withKP(flywheelKP)
        .withKD(flywheelKD)
        .withKV(flywheelKV);

        public static final SoftwareLimitSwitchConfigs hoodSoftwareLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(maxHoodAngle)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(minHoodAngle);
        
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

        public static final FeedbackConfigs feedBackConfig = new FeedbackConfigs()
        // .withFeedbackRemoteSensorID(CANCoderID)
        // .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        // .withRotorToSensorRatio(motorToEncoderGearRatio)
        .withSensorToMechanismRatio(175.0);
    }
    
    public static class TurretK {
        public static final int talonId = 15; 
        //^ This may be bad, idk if can reserves id's
        public static final int CANCoderID = 1; //! FIND
        
        
        //& CANcoder
        // public static final Angle fullRange = Degrees.of(360); //! VERIFY THIS!!!!!!!
        // public static final Angle expectedZero = Degrees.of(180); 
        // public static final Angle CANCoderOffset = Degrees.of(0); //! idk
        
        //& Gear Ratios
        public static final double krakenToTurretGearRatio = 44.5; //these numbers should be right now
        public static final double encoderToTurretGearRatio = 8.9;

        //& Motion Magic
        public static final AngularVelocity motionMagicVelocity = DegreesPerSecond.of(10); //! Find
        public static final AngularAcceleration motionMagicAcceleration = DegreesPerSecondPerSecond.of(15); //! Find

        //& Angles
        public static final Angle defaultPosition = Degrees.of(0); //^ Turret MUST be facing towards the exact front of the robot on startup. This is ESSENTIAL to zeroing. This is 
                                                                  //^ This is likely outdated with us using an absolute encoder now. Up to testing & Debugging

        // We dont go the maximum rotation to avoid wrap-around error and risk confusing the Absolute encoder, might not be needed though.
        public static final Angle maxAngle = Degrees.of(120);
        public static final Angle minAngle = Degrees.of(-155);

        //& Currents
        public static final Current maxStatorCurrent = Amps.of(0); //! Find / Verify
        public static final Current maxSupplyCurrent = Amps.of(0); //! Find / Verify

        //& PID
        public static final double kP = 60; //! Tune
        public static final double kD = 0; //! Tune
        public static final double kS = 0.19; //^ This is very wack with how the turret works. 0.19
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
        public static final int extenderCurrentLimit = 23; // amps //! Tune

        // Soft switch limits
        public static final Distance forwardSoftLimit = Inches.of(9.25); //! Tune
        public static final Distance reverseSoftLimit = Inches.of(0);

        // Voltage limits for both the wheels and the arm
        public static final Voltage spinRollerVoltage = Volts.of(-9.25); //! Tune
        public static final Voltage extendVoltage = Volts.of(-4.5); //! Tune
        public static final Voltage retractVoltage = Volts.of(5.5);

        public static final double extenderGearRatio = 11.0 / 75.6;
        public static final Distance extenderWheelDiameter = Inches.of(0.875);
    }  

    public static class IndexerK {
        public static final int id = 16; //! Find
        public static final Voltage indexingVoltage = Volts.of(6); //! Find
        public static final CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amps.of(0)) //! Find
        .withSupplyCurrentLimit(Amps.of(0)); //! Find

        public static final int sparkLeftID = 3;
        public static final int sparkRightID = 4;

        public static final int leftCurrentLimit = 0; //! Find all
        public static final int rightCurrentLimit = 0;

        public static final Voltage rightAgitatorSpinVoltage = Volts.of(-3); //! Tune
        public static final Voltage leftAgitatorSpinVoltage = Volts.of(3); //! Tune

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
        public static final Transform3d robotToLumaCamera = new Transform3d(Inches.of(-9.42), Inches.of(-10.795), Inches.of(6.6), new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-135))); //! TEST YAWS: 45, 135, -45, -135
        public static final Transform3d robotToArduCamera = new Transform3d(Inches.of(-8.947), Inches.of(6.371), Inches.of(10.494), new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(75.07)));
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
