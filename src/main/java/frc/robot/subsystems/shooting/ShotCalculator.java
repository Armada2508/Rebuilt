package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.Field;

/*``
 * https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 * https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815
 */
public class ShotCalculator {
    private final Pose2d targetPose;
    private final Pose2d robotPose;
    private final ChassisSpeeds robotVelocity;

    // private final Time latency = Seconds.of(0.5);

    private ShotParameters shotParametersInstance;
    private ShotCalculationParameters shotCalculationParametersInstance;

    public ShotCalculator(Pose2d targetPose, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        this.targetPose = targetPose;
        this.robotPose = robotPose.get();
        this.robotVelocity = robotVelocity.get();

        //^ Resets the parameter instances
        resetShotParameters();
        resetShowCalculationParameters();
    }

    public void calculate(Pose2d robotPose, ChassisSpeeds robotVelocity, Pose2d targetPosition, Time latencyCompensation) {

        //^ 1. Predicts the future position of the robot based on current velocities
        Translation2d futurePos = robotPose.getTranslation()
                                    .plus(new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                                        .times(latencyCompensation.in(Seconds))
                                    );

        //^ 2. Gets the target vector
        Translation2d toGoal = targetPosition.getTranslation().minus(futurePos);
        double distance = toGoal.getNorm(); //? Clarify what this does
        Translation2d targetDirection = toGoal.div(distance);

        //^ 3. Get baseline velocity 
        ShotCalculationParameters baseline = new ShotCalculationParameters(
            Degrees.of(Maps.getHoodAngle(distance)), 
            RotationsPerSecond.of(Maps.getFlywheelVelocity(distance)), 
            Seconds.of(Maps.getBallTimeOfFlight(Maps.getHoodAngle(distance), Maps.getFlywheelVelocity(distance)))
        );

        double baselineVelocity = distance / baseline.fuelAirTime.in(Seconds);


        double velocityRatio = requiredVelocity / baselineVelocity; //? what is this for?

        // Split correction
        double rpmFactor = Math.sqrt(velocityRatio);
        double hoodFactor = Math.sqrt(velocityRatio);

        // Apply RPM scaling
        double adjustedRpm = baseline.rpm.in(RPM) * rpmFactor;

        // Apply hood adjustment
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle.in(Degrees)));
        double targetHorizontalFromHood = baselineVelocity * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizontalFromHood / totalVelocity, 0, 1);
        double adjustedHood = Math.toDegrees(Math.acos(ratio));

        this.shotParametersInstance = new ShotParameters(Degrees.of(adjustedHood), RPM.of(adjustedRpm), Degrees.of(turretAngle));
    }

    public double getFuelHorizontalVelocity(double distance) {
        double speed = Maps.getFlywheelVelocity(distance);
        return distance / Maps.getBallTimeOfFlight(speed, 0); //! Fix the zero.
    } 

    public double velocityAndHoodAngleToEffectiveDistance(double velocity, double hoodAngle) {
        // for (Map.Entry<Double, InterpolatingTreeMap<Double, Double>> entry : Maps.fuelAirTimeMap.asMap().entrySet()) {
        for ()

    }

    //~ Getters & Resetters
    public ShotParameters getShotParameters() {
        return this.shotParametersInstance;
    }

    private void resetShotParameters() {
        this.shotParametersInstance = new ShotParameters(Degrees.of(0), RPM.of(0), Degrees.of(0));
    }

    public ShotCalculationParameters getShotCalculationParameters() {
        return this.shotCalculationParametersInstance;
    }

    private void resetShowCalculationParameters() {
        this.shotCalculationParametersInstance = new ShotCalculationParameters(Degrees.of(0), RPM.of(0), Seconds.of(0));
    }

    public record ShotCalculationParameters(
        Angle hoodAngle,
        AngularVelocity rpm,
        Time fuelAirTime
    ) {}

    public record ShotParameters(
        Angle hoodAngle,
        AngularVelocity rpm,
        Angle turretAngle
    ) {}

    // public ShotParameters getShotParameters() {
    //     //^ Gets the next predicted pose given robot velocity and multiplying.
    //     Translation2d futurePos = robotPose.getTranslation()
    //                                 .plus(
    //                                     new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
    //                                     .times(latency.in(Seconds))
    //                                 );

    //     Translation2d targetHubVector = Field.getAllianceHub().getTranslation().minus(futurePos);
    //     double distance = targetHubVector.getNorm(); //? Why are we using getNorm() and not getDistance()

    //     double idealHorizontalSpeed = getPredictedFlywheelRpm(Inches.of(distance)).in(RotationsPerSecond);

    //     Translation2d robotVelocityVector = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    //     Translation2d shotVector = targetHubVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

    //     Angle turretAngle = Degrees.of(shotVector.getAngle().getDegrees());
    //     double newHorizontalSpeed = shotVector.getNorm();

    //     double totalExitVelocity = 15.0; //! Tune? Idk how the guy got this
    //     double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
    //     Angle newHoodAngle = Degrees.of(Math.acos(ratio));

    //     return new ShotParameters(turretAngle, newHoodAngle, RotationsPerSecond.of(totalExitVelocity)); //^ This is missing the "calcRPM" form the original, idk what that is/does.
    //                                                                                                 //^ This will not work as of now because of it.
    // }


}
