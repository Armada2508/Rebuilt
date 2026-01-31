package frc.robot.subsystems.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Maps {
    /**
     * Makes an interpolating tree map for the hood angle
     */
    public static InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    /**
     * Makes an interpolating tree map for the air time
     */
    public static InterpolatingDoubleTreeMap fuelAirTimeMap = new InterpolatingDoubleTreeMap();

    /**
     * Sets the values for the interpolating tree maps
     */
    static {
        hoodAngleMap.put(0.0, 0.0); //! find
        fuelAirTimeMap.put(0.0,0.0);
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
        return fuelAirTimeMap.get(hoodAngle);
    }

    /**
     * Determines the air time from the distance determined in getHoodAngleFromDistance()
     * @param distance
     * @return
     */
    public static double getAirTimeFromDistance(double distance) {
        return fuelAirTimeMap.get(getHoodAngleFromDistance(distance));
    }

}
