package frc.robot;

import java.util.TreeMap;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Thanks to team 100 for the implementation
 * https://github.com/Team100/all26/blob/main/lib/src/main/java/org/team100/lib/util/NestedInterpolatingTreeMap.java
 * TODO: Move this into the library after the season ends
 */
public class NestedInterpolationTree<K extends Comparable<K>, V> {
    private final InverseInterpolator<K> keyInterpolator;
    private final Interpolator<V> valueInterpolator;
    private final TreeMap<K, InterpolatingTreeMap<K, V>> map;

    public NestedInterpolationTree(InverseInterpolator<K> keyInterpolator, Interpolator<V> valueInterpolator) {
        this.keyInterpolator = keyInterpolator;
        this.valueInterpolator = valueInterpolator;
        map = new TreeMap<>();
    }

    /**
     * Puts a value onto the nested map
     * @param k1 The first key it will be stored at
     * @param k2 The second key it will be stored at
     * @param value The value to store
     */
    public void put(K k1, K k2, V value) {
        map.computeIfAbsent(k1,
                (k) -> new InterpolatingTreeMap<>(
                        keyInterpolator, valueInterpolator))
                .put(k2, value);
    }

    /**
     * Retrieves a value based off of the two keys provided
     * @param k1 The first key provided
     * @param k2 The second key provided
     * @return
     */
    public V get(K k1, K k2) {
        K ceilKey1 = map.ceilingKey(k1);
        K floorKey1 = map.floorKey(k1);
        if (ceilKey1 == null && floorKey1 == null) {
            //^ map is empty
            return null;
        }
        if (ceilKey1 == null) {
            //^ k1 is beyond the top
            InterpolatingTreeMap<K, V> m = map.get(floorKey1);
            return m.get(k2);
        }
        if (floorKey1 == null) {
            //^ k1 is beyond the bottom
            InterpolatingTreeMap<K, V> m = map.get(ceilKey1);
            return m.get(k2);
        }
        InterpolatingTreeMap<K, V> floorMap1 = map.get(floorKey1);
        InterpolatingTreeMap<K, V> ceilMap1 = map.get(ceilKey1);
        
        V floorV1 = floorMap1.get(k2);
        V ceilV1 = ceilMap1.get(k2);
        double interpolatedKey1 = keyInterpolator.inverseInterpolate(
                floorKey1, ceilKey1, k1);
        return valueInterpolator.interpolate(
                floorV1, ceilV1, interpolatedKey1);
    }
    
}
