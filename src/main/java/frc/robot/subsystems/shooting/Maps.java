package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Maps {
    /**
     * Makes an interpolating tree map for the hood angle
     * (Distance, Hood Angle)
     */
    public static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    /**
     * Makes an interpolating tree map for the air time
     * (Hood Angle, Fuel Air Time)
     */
    public static InterpolatingDoubleTreeMap fuelAirTimeMap = new InterpolatingDoubleTreeMap();

    /**
     * Makes an interpolating tree map for the distance from hood angle
     * (Hood Angle, Distance)
     */
    public static InterpolatingDoubleTreeMap distanceHoodAngleMap = new InterpolatingDoubleTreeMap();

    /**
     * Sets the values for the interpolating tree maps
     */
    static {
        // hoodAngleMap.put(0.0, 0.0); //! find
        // hoodAngleMap.put(0.5, 3.3);
        // hoodAngleMap.put(1.0, 6.6);
        // hoodAngleMap.put(1.5, 9.9);
        // hoodAngleMap.put(2.0, 13.2);
        // hoodAngleMap.put(2.5, 16.5);
        // hoodAngleMap.put(3.0, 19.8);
        // hoodAngleMap.put(3.5, 23.1);
        // hoodAngleMap.put(4.0, 26.4);
        // hoodAngleMap.put(4.5, 29.7);
        // hoodAngleMap.put(5.0, 33.0);
        // hoodAngleMap.put(5.5, 36.3);
        // hoodAngleMap.put(6.0, 39.6);

        // fuelAirTimeMap.put(0.0,0.0);
        // distanceHoodAngleMap.put(0.0, 0.0);
    }
    
    /**
     * Determines the hood angle from the distance to the target
     * @param distance
     * @return
     */
    public static Angle getHoodAngleFromDistance(Distance distance) {
        return Degrees.of(hoodAngleMap.get(distance.in(Meters)));
    }

    /**
     * Determines the air time to the target from the hood angle
     * @param hoodAngle
     * @return
     */
    public static double getAirTimeFromHoodAngle(double hoodAngle) {
        return fuelAirTimeMap.get(hoodAngle);
    }

    /**
     * Determines the air time from the distance determined in getHoodAngleFromDistance()
     * @param distance
     * @return
     */
    // public static double getAirTimeFromDistance(double distance) {
    //     return fuelAirTimeMap.get(getHoodAngleFromDistance(distance));
    // }
    /**
     * Determines the distance from the hood angle
     * @param hoodAngle
     * @return
     */
    public static double getDistanceFromHoodAngle(double hoodAngle) {
        return distanceHoodAngleMap.get(hoodAngle);
    }

}
