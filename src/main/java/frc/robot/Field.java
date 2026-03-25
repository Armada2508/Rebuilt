package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Field {
    
    public static final Pose2d origin = Pose2d.kZero;

    // Field
    public static final Distance fieldLength = Inches.of(651.2);
    public static final Distance fieldWidth = Inches.of(317.7);
    public static final Pose2d fieldCenter = new Pose2d(fieldLength.div(2), fieldWidth.div(2), Rotation2d.kZero);

    // Blue side
    public static final Pose2d blueHub = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(158.84)), Rotation2d.k180deg); 
    public static final Pose2d blueTower = new Pose2d(new Translation2d(Inches.of(41.56), Inches.of(147.47)), Rotation2d.kZero);
    public static final Pose2d blueDepot = new Pose2d(new Translation2d(Inches.of(15.5), Inches.of(234.77)), Rotation2d.kZero); 
    public static final Pose2d blueOutpost = new Pose2d(new Translation2d(Inches.of(0), Inches.of(26.22)), Rotation2d.kZero);
    public static final Pose2d blueTrenchRight = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(24.92)), Rotation2d.k180deg);
    public static final Pose2d blueTrenchLeft = new Pose2d(new Translation2d(blueTrenchRight.getMeasureX(), fieldWidth.minus(blueTrenchRight.getMeasureY())), Rotation2d.k180deg); 
    public static final Pose2d passTargetBlueHigh = new Pose2d(new Translation2d(blueHub.getMeasureX().times(0.60), blueHub.getMeasureY().times(0.375)), Rotation2d.kZero);
    public static final Pose2d passTargetBlueLow = new Pose2d(new Translation2d(blueHub.getMeasureX().times(0.60), blueHub.getMeasureY().times(1.625)), Rotation2d.kZero);
    public static final Pose2d blueZoneCorner1 = new Pose2d(Inches.of(0),Inches.of(0), Rotation2d.kZero);
    public static final Pose2d blueZoneCorner2 = new Pose2d(Inches.of(156.61), Inches.of(317.69), Rotation2d.kZero);
    public static final Rectangle2d blueZone = new Rectangle2d(new Translation2d(0,0), new Translation2d(156.61, 317.69));

    // Red side
    public static final Pose2d redHub = new Pose2d(new Translation2d(fieldLength.minus(blueHub.getMeasureX()), (fieldWidth.minus(blueHub.getMeasureY()))), Rotation2d.kZero); 
    public static final Pose2d redTower = new Pose2d(new Translation2d(fieldLength.minus(blueTower.getMeasureX()), (fieldWidth.minus(blueTower.getMeasureY()))), Rotation2d.k180deg);
    public static final Pose2d redDepot = new Pose2d(new Translation2d(fieldLength.minus(blueDepot.getMeasureX()), (fieldWidth.minus(blueDepot.getMeasureY()))), Rotation2d.k180deg);
    public static final Pose2d redOutpost = new Pose2d(new Translation2d(fieldLength.minus(blueOutpost.getMeasureX()), (fieldWidth.minus(blueOutpost.getMeasureY()))), Rotation2d.k180deg);
    public static final Pose2d redTrenchRight = new Pose2d(new Translation2d(fieldLength.minus(blueTrenchRight.getMeasureX()), (fieldWidth.minus(blueTrenchRight.getMeasureY()))), Rotation2d.kZero);
    public static final Pose2d redTrenchLeft = new Pose2d(new Translation2d(fieldLength.minus(blueTrenchLeft.getMeasureX()), (fieldWidth.minus(blueTrenchLeft.getMeasureY()))), Rotation2d.kZero);
    public static final Pose2d passTargetRedHigh = new Pose2d(new Translation2d(redHub.getMeasureX().times(1.15), redHub.getMeasureY().times(0.375)), Rotation2d.kZero);
    public static final Pose2d passTargetRedLow = new Pose2d(new Translation2d(redHub.getMeasureX().times(1.15), redHub.getMeasureY().times(1.625)), Rotation2d.kZero);
    public static final Pose2d redZoneCorner1 = new Pose2d(Inches.of(651.22),Inches.of(317.69), Rotation2d.kZero);
    public static final Pose2d redZoneCorner2 = new Pose2d(Inches.of(469.11),Inches.of(0), Rotation2d.kZero);
    public static final Rectangle2d redZone = new Rectangle2d(new Translation2d(651.22,0), new Translation2d(469.11, 317.69));
    
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
     * Get the hub depending on your alliance
     * @return The alliances hub
     */
    public static Pose2d getAllianceHub() {
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) return blueHub;
        return redHub; // If on red, return red hub 
    }

    public static Distance getDistanceToHub(Pose2d pose) {
        Translation2d hubTranslation = getAllianceHub().getTranslation();
        return Meters.of(pose.getTranslation().getDistance(hubTranslation));
    }
}

