package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/*
 * https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 * https://www.chiefdelphi.com/t/shoot-on-the-move-from-the-code-perspective/511815
 */
public class ShotCalculator {
    private Pose2d targetPose;
    private ChassisSpeeds robotVelocity; //! This must be field-centric velocity. Not robot-centric
    private Pose2d robotPose;

    private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRpmMap = new InterpolatingDoubleTreeMap();

    private Time latency = Seconds.of(0.15); 


    static {
        hoodAngleMap.put(0.0, 0.0);
        //! Tune the rest of the map

        flywheelRpmMap.put(0.0, 0.0);
        //! Tune the rest of the map

    }

    /**
     * Return the predicted hood angle based off of how far you are from the hub
     * @param distance
     * @return
     */
    public Angle getPredictedHoodAngle(Distance distance) {
        return Degrees.of(hoodAngleMap.get(distance.in(Inches)));
    }

    /**
     * Return the predicted flywheel RPM based off of how far you are from the hub
     * @param distance
     * @return
     */
    public AngularVelocity getPredictedFlywheelRpm(Distance distance) {
        return RotationsPerSecond.of(flywheelRpmMap.get(distance.in(Inches)));
    }


    public ShotCalculator(Pose2d targetPose, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotVelocity) {
        this.targetPose = targetPose;
        this.robotPose = robotPose.get();
        this.robotVelocity = robotVelocity.get();
    }

    public ShotParameters getShotParameters() {
        //^ Gets the next predicted pose given robot velocity and multiplying.
        Translation2d futurePos = robotPose.getTranslation()
                                    .plus(
                                        new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                                        .times(latency.in(Seconds))
                                    );

        Translation2d targetHubVector = Field.getAllianceHub().getTranslation().minus(futurePos);
        double distance = targetHubVector.getNorm(); //? Why are we using getNorm() and not getDistance()

        double idealHorizontalSpeed = getPredictedFlywheelRpm(Inches.of(distance)).in(RotationsPerSecond);

        Translation2d robotVelocityVector = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
        Translation2d shotVector = targetHubVector.div(distance).times(idealHorizontalSpeed).minus(robotVelocityVector);

        Angle turretAngle = Degrees.of(shotVector.getAngle().getDegrees());
        double newHorizontalSpeed = shotVector.getNorm();

        double totalExitVelocity = 15.0; //! Tune? Idk how the guy got this
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        Angle newHoodAngle = Degrees.of(Math.acos(ratio));

        return new ShotParameters(turretAngle, newHoodAngle, RotationsPerSecond.of(totalExitVelocity)); //^ This is missing the "calcRPM" form the original, idk what that is/does.
                                                                                                    //^ This will not work as of now because of it.
    }

    public record ShotParameters(
        Angle turretAngle,
        Angle hoodAngle,
        AngularVelocity rpm
    ) {}
}
