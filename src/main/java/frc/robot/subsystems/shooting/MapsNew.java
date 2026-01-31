package frc.robot.subsystems.shooting;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class MapsNew {
    /**
     * Makes an interpolating tree map for the hood angle
     */
    public static InterpolatingTreeMap<Double, Double> hoodAngleMap = 
    new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    /**
     * Makes an interpolating tree map for the air time
     */
    public static InterpolatingTreeMap<Double, Double> airTimeMap = 
    new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    /**
     * Sets the values for the interpolating tree maps
     */
    static {
        hoodAngleMap.put(0.0, 0.0); //! find
        airTimeMap.put(0.0,0.0);
    }
    
    /**
     * Determines the hood angle from the distance to the target
     * @param distance
     * @return
     */
    public static double getHoodAngleFromDistance(double distance) {
        return hoodAngleMap.get(distance);
    }

    /**
     * Determines the air time to the target from the hood angle
     * @param hoodAngle
     * @return
     */
    public static double getAirTimeFromHoodAngle(double hoodAngle) {
        return airTimeMap.get(hoodAngle);
    }

    /**
     * Determines the air time from the distance determined in getHoodAngleFromDistance()
     * @param distance
     * @return
     */
    public static double getAirTimeFromDistance(double distance) {
        return airTimeMap.get(getHoodAngleFromDistance(distance));
    }

}

