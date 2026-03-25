package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
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
        rpmMap.put(0.0, 0.0);

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
