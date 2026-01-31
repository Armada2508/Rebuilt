package frc.robot.subsystems.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class MapsNew {
    public static InterpolatingTreeMap<Double, ShooterParams> hoodAngleMap = 
    new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    
    static {
        hoodAngleMap.put(1.5, new ShooterParams(0.0, 0.0));
    }

    // Simple data type for parameters
    public record ShooterParams(
        double hoodAngle,
        double airTime
    ) {}
}

