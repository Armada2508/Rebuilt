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

    //~ Variables & Objects
    ShotCalculator shotCalculatorPass = new ShotCalculator();
    ShotCalculator shotCalculatorScore = new ShotCalculator();
    public static ShotParameters passCalculations;
    public static ShotParameters scoreCalculations;
    

    //~ Subsystems
    private static Shooter shooter = new Shooter();
    private static Turret turret = new Turret();

    //~ Methods
    /**
     * Sets the Turret's angle to 0 and the Shooter's hood to it's lowest possible angle.
     */
    public void zero() {
        shooter.setHoodAngle(ShooterK.minHoodAngle);
        turret.setAngleCommand(Degrees.of(0));
    }

    /**
     * Method that tells us which Hub the <STRONG>score()</STRONG> command should target.
     * @return Pose2d of our alliance's hub.
     */
    public static Pose2d whichHub() {
        Pose2d hub = (DriverStation.getAlliance().get().equals(Alliance.Blue)) ? (hub = Field.blueHub) : (hub = Field.redHub); // yro'ue welcome chris
        return hub;
    }   

    //~ Commands
    @Logged
    public void periodic(Pose2d robotPose, ChassisSpeeds velocity, Time latency) { //! UPDATE WHEN SWERVE IS MERGED
        shotCalculatorPass.resetShotCalculationParameters();  
        shotCalculatorScore.resetShotCalculationParameters(); 
        shotCalculatorPass.calculate(robotPose, velocity, Field.getClosestPassPoint(robotPose), latency);
        shotCalculatorScore.calculate(robotPose, velocity, whichHub(), latency);
        passCalculations = shotCalculatorPass.getShotParameters();
        scoreCalculations = shotCalculatorScore.getShotParameters();
    }

    /**
     * Command to score fuel into the hub.
     * @return
     */
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
    
}
