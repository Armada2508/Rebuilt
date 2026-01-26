package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;

import frc.robot.Constants.HopperK;
import frc.robot.lib.util.Util;


import com.playingwithfusion.TimeOfFlight;

public class Hopper {
    
    private final TimeOfFlight timeOfFlightTop = new TimeOfFlight(HopperK.timeOfFlightIdTop);
    private final TimeOfFlight timeOfFlightBottom = new TimeOfFlight(HopperK.timeOfFlightIdBottom);
    public boolean isTOFRangeValid() {
        return timeOfFlightTop.isRangeValid();
    }

    public boolean isTOFBottomRangeValid() {
        return timeOfFlightBottom.isRangeValid();
    }

    public boolean isHopperFull() {
        return Util.inRange(timeOfFlightTop.getRange(), HopperK.hopperDetectionRange.in(Inches));
    }
    
    public boolean isThereFuel() {
        return Util.inRange(timeOfFlightBottom.getRange(), HopperK.hopperDetectionRange.in(Inches));
    }
}
