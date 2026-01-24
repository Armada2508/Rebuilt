package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.Degrees;
public class Constants {

    public static class ShooterK { //! find motor ID and proper measurements
        public static final int talonID = 0;
        public static final int sparkmaxHoodID = 1;

        public static final Voltage fuelShootVoltage = Volts.of(1);
        public static final Time flywheelSpeedUpTime = Seconds.of(0.5);
        public static final AngularVelocity cruiseVelocity = DegreesPerSecond.of(0);
        public static final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(0);

        public static final Angle minHoodAngle = Degrees.of(0); //! FIND
        public static final Angle maxHoodAngle = Degrees.of(0); //! FIND
    }
}