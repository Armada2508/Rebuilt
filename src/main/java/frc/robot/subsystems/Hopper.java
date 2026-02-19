package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import frc.robot.Constants.HopperK;
import frc.robot.lib.util.Util;


import com.playingwithfusion.TimeOfFlight;

public class Hopper {
    
    private final TimeOfFlight timeOfFlightTop = new TimeOfFlight(HopperK.timeOfFlightIdTop);
    private final TimeOfFlight timeOfFlightBottom = new TimeOfFlight(HopperK.timeOfFlightIdBottom);
    /**
     * Checks if the range of the top TOF sensor has a valid measurement after it measures a distance
     * @return
     */
    public boolean isTOFTopRangeValid() {
        return timeOfFlightTop.isRangeValid();
    }
    /**
     * Checks if the range of the bottom TOF sensor has a valid measurement after it measures a distance
     * @return
     */
    public boolean isTOFBottomRangeValid() {
        return timeOfFlightBottom.isRangeValid();
    }
    /**
     * Detects if the hopper if full
     * @return
     */
    public boolean isFull() {
        return Util.inRange(timeOfFlightTop.getRange(), HopperK.hopperTopDetectionRange.in(Inches));
    }
    /**
     * Detects if there is fuel in the hopper
     * @return
     */
    public boolean hasFuel() {
        return Util.inRange(timeOfFlightBottom.getRange(), HopperK.hopperBottomDetectionRange.in(Inches));
    }
}
