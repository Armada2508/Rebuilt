package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.SwerveK;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.motors.TalonFXSwerve;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    private final SwerveDrive swerveDrive;

    private final TalonFX frontLeft;
    private final TalonFX frontRight;
    private final TalonFX backLeft;
    private final TalonFX backRight;
    private final SysIdRoutine sysIdRoutine; 
    private final PPHolonomicDriveController pathPlannerController = new PPHolonomicDriveController(SwerveK.ppTranslationConstants, SwerveK.ppRotationConstants);
    private boolean initializedOdometryFromVision = false;
    @SuppressWarnings("unused")
    private Pose2d pathPlannerTarget = Pose2d.kZero; // For logging
    private final BooleanSupplier overridePathFollowing;
    private final Debouncer overrideDebouncer = new Debouncer(ControllerK.overrideTime.in(Seconds));
    private Pose2d targetPose;
    private boolean completedAlignmentBool = false;
    public final Trigger completedAlignment = new Trigger(() -> completedAlignmentBool);
    private final ProfiledPIDController xController = new ProfiledPIDController(SwerveK.translationConstants.kP, SwerveK.translationConstants.kI, SwerveK.translationConstants.kD, SwerveK.defaultTranslationConstraints);
    private final ProfiledPIDController yController = new ProfiledPIDController(SwerveK.translationConstants.kP, SwerveK.translationConstants.kI, SwerveK.translationConstants.kD, SwerveK.defaultTranslationConstraints);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(SwerveK.rotationConstants.kP, SwerveK.rotationConstants.kI, SwerveK.rotationConstants.kD, SwerveK.defaultRotationConstraints);


    public Swerve(BooleanSupplier overridePathFollowing) {
                this.overridePathFollowing = overridePathFollowing;
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        SwerveParser parser = null;
        try {
            parser = new SwerveParser(SwerveK.swerveDirectory);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Swerve directory not found.");
        }
        swerveDrive = parser.createSwerveDrive(SwerveK.maxPossibleRobotSpeed.in(MetersPerSecond));
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(SwerveK.kS, SwerveK.kV, SwerveK.kA));
        frontLeft = (TalonFX) swerveDrive.getModules()[0].getDriveMotor().getMotor();
        frontRight = (TalonFX) swerveDrive.getModules()[1].getDriveMotor().getMotor();
        backLeft = (TalonFX) swerveDrive.getModules()[2].getDriveMotor().getMotor();
        backRight = (TalonFX) swerveDrive.getModules()[3].getDriveMotor().getMotor();
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    VoltageOut request = new VoltageOut(volts);
                    for (var module : swerveDrive.getModules()) {
                        var motor = (TalonFXSwerve) module.getDriveMotor();
                        ((TalonFX) motor.getMotor()).setControl(request);
                    }
                },
                null,
                this
            )
        );

        setupPathPlanner();
        frontLeft.getConfigurator().apply(SwerveK.currentLimitsConfig);
        frontRight.getConfigurator().apply(SwerveK.currentLimitsConfig);
        backLeft.getConfigurator().apply(SwerveK.currentLimitsConfig);
        backRight.getConfigurator().apply(SwerveK.currentLimitsConfig);


    }

        private void setupPathPlanner() {
        AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry, 
            this::getChassisSpeeds, 
            (speeds, feedforward) -> setChassisSpeeds(speeds), 
            pathPlannerController,
            SwerveK.robotConfig,
            Robot::onRedAlliance, 
            this);
    }

    /**
     * Resets the odometry to the given pose
     * @param pose Pose to reset the odemetry to
     */
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Returns the robot's pose
     * @return Current pose of the robot as a Pose2d
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }
    
    /**
     * Returns the robot's velocity (x, y, and omega)
     * @return Current velocity of the robot
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Set the speed of the robot with closed loop velocity control
     * @param chassisSpeeds to set speed with (robot relative)
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }


}
