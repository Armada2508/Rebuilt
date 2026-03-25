// package frc.robot.subsystems.shooting;

// import static edu.wpi.first.units.Units.Degrees;

// import edu.wpi.first.epilogue.Logged;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.measure.Time;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ShooterK;

// //~ Subsystems
// import frc.robot.subsystems.shooting.ShotCalculator.ShotParameters;
// import frc.robot.Field;

// @Logged
// public class Superstructure extends SubsystemBase {
//     // ^ This class needs to have all of shooter and turret done in order to be made

//     //~ Variables & Objects
//     ShotCalculator shotCalculatorPass = new ShotCalculator();
//     ShotCalculator shotCalculatorScore = new ShotCalculator();
//     public static ShotParameters passParameters;
//     public static ShotParameters scoreParameters;
    
//     //~ Subsystems
//     private Shooter shooter;
//     private Turret turret;

//     public Superstructure(Shooter shooter, Turret turret) {
//         this.shooter = shooter;
//         this.turret = turret;
//     }

//     //~ Methods
//     /**
//      * Sets the Turret's angle to 0 and the Shooter's hood to it's lowest possible angle.
//      */
//     public void zero() {
//         // shooter.setHoodAngle(ShooterK.minHoodAngle);
//         turret.setAngleCommand(Degrees.of(0));
//     }
  
//     //~ Commands

//     public void periodic(Pose2d robotPose, ChassisSpeeds velocity, Time latency) {
//         shotCalculatorPass.resetShotCalculationParameters();  
//         shotCalculatorScore.resetShotCalculationParameters(); 
//         shotCalculatorPass.calculate(robotPose, velocity, Field.getClosestPassPoint(robotPose), latency);
//         shotCalculatorScore.calculate(robotPose, velocity, Field.getAllianceHub(), latency);
//         passParameters = shotCalculatorPass.getShotParameters();
//         scoreParameters = shotCalculatorScore.getShotParameters();
//     }

//     /**
//      * Command to score fuel into the hub.
//      * @return
//      */
//     // public Command score() {
//     //     return runOnce(() ->
//     //         shooter.setHoodAngle(scoreParameters.hoodAngle())
//     //         .alongWith(
//     //             turret.setAngleCommand(scoreParameters.turretAngle()))
//     //     ).andThen(shooter.shootFuel());
//     // }

//     /**
//      * Command used to shoot fuel to the closest of one of two passpoints.
//      * @return 
//      */
//     // public Command pass() {
//     //     return runOnce(() -> 
//     //         shooter.setHoodAngle(passParameters.hoodAngle())
//     //         .alongWith(
//     //             turret.setAngleCommand(passParameters.turretAngle()))
//     //     ).andThen(shooter.shootFuel());
//     // }

//     /**
//      * Returns the currently running command
//      * @return The command being run
//      */
//     @Logged(name = "Current Command")
//     public String getCurrentCommandName() {
//         var cmd = getCurrentCommand();
//         if (cmd == null) return "None";
//         return cmd.getName();
//     }
    
//     /**
//      * Command that sets the shooter to point towards the Hub.
//      * @return
//      */
//     //public Command trackHub() {
//     //}
//         //^ May not need due to the program's method of retriving data (shotCalculator)
// }
