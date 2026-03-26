package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class Maps {
    /**
     * Makes an interpolating tree map for the hood angle
     * (Distance, RPM)
     */
    public static InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    /**
     * Sets the values for the interpolating tree maps
     */
    static {
        rpmMap.put(1.75, 2650.0);
        rpmMap.put(2.0, 2700.0);
        rpmMap.put(2.25, 2750.0);
        rpmMap.put(2.5, 2825.0);
        rpmMap.put(2.75, 2900.0);
        rpmMap.put(3.0, 3000.0);
        rpmMap.put(3.25, 3100.0);
        rpmMap.put(3.5, 3225.0);
        rpmMap.put(3.75, 3350.0);
        rpmMap.put(4.0, 3500.0);
        rpmMap.put(4.25, 3650.0);
        rpmMap.put(4.5, 3825.0);

    }
    
    /**
     * Determines the rpm from the distance to the target
     * @param distance to the target
     * @return interpolated rpm
     */
    public static AngularVelocity getRpmFromDistance(Supplier<Distance> distance) {
        return RPM.of(rpmMap.get(distance.get().in(Meters)));
    }
}
