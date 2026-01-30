package frc.robot.subsystems.shooting;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.NestedInterpolationTree;
/**
 * 2d Tree Map: https://github.com/Team100/all26/blob/main/lib/src/main/java/org/team100/lib/util/NestedInterpolatingTreeMap.java
 */

public class Maps {
    /**
     * Prevent this class from being instantiated because of private access modifier.
     */
    private Maps() {}

    private static final double defaultHoodAngle = 0.0; //! Find and move into constants
    private static final double defaultFlywheelRpm = 0.0; //! Find and move into constants
    private static final double defaultDistance = 0.0; //! Find and move into constants 
                                                       //? Might not be needed depending on implementation

    //~ Create the maps
    // Distance (meters) -> Hood Angle (degrees)
    public static final InterpolatingTreeMap<Double, Double> hoodAngleMap 
        = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    // Distance (meters) -> Flywheel Velocity (rpm) //? Maybe RPS?
    public static final InterpolatingTreeMap<Double, Double> flywheelRpmMap 
        = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    // {Flywheel Velocity (rpm), Hood Angle (degrees)} -> Time of Flight (seconds)
    //^ The two interpolating values MUST be the same, that is why there is only 2 parameters despite being a table of 3. The first Double represents both of the interpolating axises
    public static final NestedInterpolationTree<Double, Double> fuelAirTimeMap 
        = new NestedInterpolationTree<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    static {
        //~ Fill Hood Angle Map
        //^ This map assumes a static RPM to fire at, use defaultFlywheelRpm
        hoodAngleMap.put(0.0, 0.0);
        //! Fill the rest of this map, atleast 8-12 pairs

        //~ Fill Flywheel RPM Map
        //^ This map assumes a static hood angle to fire at, use defaultFlywheelRpm
        hoodAngleMap.put(0.0, 0.0);
        //! Fill the rest of this map, atleast 8-12 pairs
        
        //~ Fill Time of Flight Map
        //^ This map assumes a static distance to fire at.
        fuelAirTimeMap.put(0.0, 0.0, 0.0);
        //! Fill the rest of this map, it should make a checkerboard like structure. Aim for an 8x8 - 12x12.
    }

    /**
     * Gets the ideal hood angle depending on the distance from the target
     * @param distance distance to interpolate from
     * @return
     */
    public static double getHoodAngle(double distance) {
        return hoodAngleMap.get(distance);
    }

    /**
     * Gets the ideal flywheel velocity depending on the distance from the target
     * @param distance distance to interpolate from
     * @return
     */
    public static double getFlywheelVelocity(double distance) {
        return flywheelRpmMap.get(distance);
    }

    /**
     * Gets the time the projectile is in the air for depending on the rpm and hood angle of the shooter
     * @param rpm The rpm the shooter's flywheel is at
     * @param hoodAngle The angle the shooter's hood is at
     * @return
     */
    public static double getBallTimeOfFlight(double rpm, double hoodAngle) {
        return fuelAirTimeMap.get(rpm, hoodAngle);
    }

}
