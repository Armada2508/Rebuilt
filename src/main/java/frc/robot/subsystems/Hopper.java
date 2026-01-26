package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import frc.robot.Constants.HopperK;
import frc.robot.lib.util.Util;


import com.playingwithfusion.TimeOfFlight;

public class Hopper {
    
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(HopperK.timeOfFlightId);


    
    public boolean isTOFRangeValid() {
        return timeOfFlight.isRangeValid();
    }

    

    
    public boolean isSensorTripped() {
        return Util.inRange(timeOfFlight.getRange(), HopperK.hopperDetectionRange.in(Millimeters));
    }
}
