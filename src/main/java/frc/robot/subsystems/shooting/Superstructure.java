package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;
import frc.robot.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

//~ Subsystems
import frc.robot.subsystems.shooting.Shooter;
import frc.robot.subsystems.shooting.Turret;
import frc.robot.subsystems.shooting.ShotCalculator.ShotParameters;
import frc.robot.subsystems.shooting.ShotCalculator;
import frc.robot.subsystems.shooting.Maps;
import frc.robot.Field;
import frc.robot.subsystems.Vision;

public class Superstructure {
    // ^ This class needs to have all of shooter and turret done in order to be made

    //~ Variables

    private final static Pose2d passTargetBlueHigh = Field.passTargetBlueHigh;
    private final static Pose2d passTargetBlueLow = Field.passTargetBlueLow;
    private final static Pose2d passTargetRedHigh = Field.passTargetRedHigh;
    private final static Pose2d passTargetRedLow = Field.passTargetRedLow;

    public static ShotParameters nerdystats;
    /**
     * Figures out the closest passing point from the robot's location.
     * 
     * @return 
     */
    public static Pose2d getClosestPoint(Pose2d robotPose) {
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            double distanceBlueHigh = passTargetBlueHigh.getTranslation().getDistance(robotPose.getTranslation());
            double distanceBlueLow = passTargetBlueLow.getTranslation().getDistance(robotPose.getTranslation());
            
            if (distanceBlueHigh <= distanceBlueLow) {
                return passTargetBlueHigh;
            }
            else {
                return passTargetBlueLow;
            }
        }
        else {
            double distanceRedHigh = passTargetRedHigh.getTranslation().getDistance(robotPose.getTranslation());
            double distanceRedLow = passTargetRedLow.getTranslation().getDistance(robotPose.getTranslation());

            if (distanceRedHigh <= distanceRedLow) {
                return passTargetRedHigh;
            }
            else {
                return passTargetRedLow;
            }
        }
        
    }

    public void periodic(Pose2d robotPose, ChassisSpeeds velocity, Time latency) { //! UPDATE WHEN SWERVE IS MERGED
        ShotCalculator shotCalculator = new ShotCalculator(); 
        shotCalculator.calculate(robotPose, null, getClosestPoint(robotPose), latency);
        nerdystats = shotCalculator.getShotParameters();
    }


    public Command score() {
        
    }

    public Command Pass(AngularVelocity rpm) {
        Shooter.setHoodAngle(nerdystats.hoodAngle());
        return;
    }

    public Command trackHub() {
        // May not need due to the program's method of retriving data (shotCalculator)
    }

    public Command zero() {

    }
}
