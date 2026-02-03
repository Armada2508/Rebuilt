package frc.robot.subsystems.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;


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
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    private final static Pose2d blueHub = Field.blueHub;
    private final static Pose2d redHub = Field.redHub;

    ShotCalculator shotCalculatorPass = new ShotCalculator();
    ShotCalculator shotCalculatorScore = new ShotCalculator();
    public static ShotParameters passCalculations;
    public static ShotParameters scoreCalculations;
    

    //~ Subsystems

    private static Shooter shooter = new Shooter();
    private static Turret turret = new Turret();

    /**
     * Figures out the closest passpoint from the robot's location.
     * 
     * @return The Pose2d of the closest passpoint of the robot's alliance.
     */
    public static Pose2d getClosestPassPoint(Pose2d robotPose) {
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

    /**
     * Method that tells us which Hub the <STRONG>score()</STRONG> command should target.
     * @return Pose2d of our alliance's hub.
     */
    public static Pose2d whichHub() {
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return blueHub;  
        }
        else {
            return redHub;
        }
    }   
    @Logged
    public void periodic(Pose2d robotPose, ChassisSpeeds velocity, Time latency) { //! UPDATE WHEN SWERVE IS MERGED
        passCalculations.resetShotCalculationParameters();  
        scoreCalculations.resetShotCalculationParameters(); 
        shotCalculatorPass.calculate(robotPose, velocity, getClosestPassPoint(robotPose), latency);
        shotCalculatorScore.calculate(robotPose, velocity, whichHub(), latency);
        passCalculations = shotCalculatorPass.getShotParameters();
        scoreCalculations = shotCalculatorScore.getShotParameters();
    }


    public Command score() {
        return runOnce(() ->
        shooter.setHoodAngle(scoreCalculations.hoodAngle())
        .alongWith(
            turret.setAngleCommand(scoreCalculations.turretAngle()))
            .andThen(shooter.shootFuel(ShooterK.staticRpm))
        );
    }
    /**
     * Command used to shoot fuel to the closest of one of two passpoints.
     * @return 
     */
    public Command pass() {
        return runOnce(() -> 
            shooter.setHoodAngle(passCalculations.hoodAngle())
            .alongWith(
            turret.setAngleCommand(passCalculations.turretAngle()))
            .andThen(shooter.shootFuel(ShooterK.staticRpm))
        );
    }
    /**
     * Command that sets the shooter to point towards the Hub.
     * @return
     */
    public Command trackHub() {
        // May not need due to the program's method of retriving data (shotCalculator)
    }
    /**
     * 
     */
    public void zero() {
        shooter.setHoodAngle(ShooterK.minHoodAngle);
        turret.setAngleCommand(Degrees.of(0));
    }
}
