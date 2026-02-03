package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.ShooterK;

/*``
 * https://blog.eeshwark.com/robotblog/shooting-on-the-fly-pt2
 * https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815
 */
public class ShotCalculator {
    // private final Pose2d targetPose;
    // private final Pose2d robotPose;
    // private final ChassisSpeeds robotVelocity;

    // private final Time latency = Seconds.of(0.5);

    private ShotParameters shotParametersInstance;
    private ShotCalculationParameters shotCalculationParametersInstance;

    public ShotCalculator(/* Pose2d targetPose, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity*/ ) {
        // this.targetPose = targetPose;
        // this.robotPose = robotPose.get();
        // this.robotVelocity = robotVelocity.get();

        //^ Resets the parameter instances
        resetShotParameters();
        resetShotCalculationParameters();
    }

    public void calculate(Pose2d robotPose, ChassisSpeeds robotVelocity, Pose2d targetPosition, Time latencyCompensation) {
        Translation2d robotVelocityVector = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
        

        //^ 1. Predicts the future position of the robot based on current velocities
        Translation2d futurePos = robotPose.getTranslation()
                                    .plus(robotVelocityVector
                                        .times(latencyCompensation.in(Seconds))
                                    );

        //^ 2. Gets the target vector
        Translation2d toGoal = targetPosition.getTranslation().minus(futurePos);
        double distance = toGoal.getNorm(); //? Clarify what this does
        Translation2d targetDirection = toGoal.div(distance); // Makes a lengthless vector

        //^ 3. Get baseline state
        ShotCalculationParameters baseline = new ShotCalculationParameters(
            Degrees.of(Maps.getHoodAngleFromDistance(distance)), // Baseline hood angle
            Seconds.of(Maps.getAirTimeFromDistance(distance)) // Baseline air time
        );
        double baselineFuelVelocity = distance / baseline.fuelAirTime.in(Seconds); // Velocity of the ball

        //^ 4a. Build target velocity vector
        Translation2d targetVelocityVector = targetDirection.times(baselineFuelVelocity);

        //^ 4b. Subtract robot velocity
        Translation2d shotVelocityVector = targetVelocityVector.minus(robotVelocityVector);

        double turretAngle = shotVelocityVector.getAngle().getDegrees();
        double horizontalVelocityRequired = shotVelocityVector.getNorm();

        //^ 5. Find total exit velocity of the ball
        double totalVelocity = baselineFuelVelocity / Math.cos(Math.toRadians(baseline.hoodAngle.in(Degrees)));
        double effectiveDistance = Maps.getHoodAngleFromDistance(horizontalVelocityRequired); //! Double check this is correct
        double horizontalVelocityFromHood = Maps.getHoodAngleFromDistance(effectiveDistance);

        //^ 5a. Find hood target to achieve total exit velocity
        double ratio = MathUtil.clamp(horizontalVelocityFromHood / totalVelocity, 0, 1);
        double adjustedHood = Math.toDegrees(Math.acos(ratio));
        adjustedHood = MathUtil.clamp(adjustedHood, ShooterK.minHoodAngle.in(Degrees), ShooterK.maxHoodAngle.in(Degrees));

        //^ 6. Store final parameters
        this.shotParametersInstance = new ShotParameters(Degrees.of(adjustedHood), Degrees.of(turretAngle));
    }

    //? I do not believe we still need this method?
    // public double getFuelHorizontalVelocity(double distance) {
    //     double speed = Maps.getFlywheelVelocity(distance);
    //     return distance / Maps.getBallTimeOfFlight(speed, 0); //! Fix the zero.
    // } 

    //? ditto?
    // public double velocityAndHoodAngleToEffectiveDistance(double velocity, double hoodAngle) {
    //     // for (Map.Entry<Double, InterpolatingTreeMap<Double, Double>> entry : Maps.fuelAirTimeMap.asMap().entrySet()) {
    //     for ()

    // }

    //~ Getters & Resetters
    public ShotParameters getShotParameters() {
        return this.shotParametersInstance;
    }

    private void resetShotParameters() {
        this.shotParametersInstance = new ShotParameters(Degrees.of(0), Degrees.of(0));
    }

    public ShotCalculationParameters getShotCalculationParameters() {
        return this.shotCalculationParametersInstance;
    }

    private void resetShotCalculationParameters() {
        this.shotCalculationParametersInstance = new ShotCalculationParameters(Degrees.of(0), Seconds.of(0));
    }

    public record ShotCalculationParameters(
        Angle hoodAngle,
        Time fuelAirTime
    ) {}

    public record ShotParameters(
        Angle hoodAngle,
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
