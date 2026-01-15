package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class Field {
    // X is left to right, Y is top to bottom

    public static final Pose2d origin = Pose2d.kZero;

    // Field
    public static final Distance fieldLength = Inches.of(651.2);
    public static final Distance fieldWidth = Inches.of(317.7);
    public static final Pose2d fieldCenter = new Pose2d(fieldLength.div(2), fieldWidth.div(2), Rotation2d.kZero);

    // Blue side
    public static final Pose2d blueHub = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(158.84)), Rotation2d.k180deg); 
    public static final Pose2d blueTower = new Pose2d(new Translation2d(Inches.of(41.56), Inches.of(147.47)), Rotation2d.kZero);
    public static final Pose2d blueDepot = new Pose2d(new Translation2d(Inches.of(15.5), Inches.of(234.77)), Rotation2d.kZero); //! This is off
    public static final Pose2d blueOutpost = new Pose2d(new Translation2d(Inches.of(0), Inches.of(26.22)), Rotation2d.kZero);
    public static final Pose2d blueTrenchBottom = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(24.92)), Rotation2d.k180deg);
    public static final Pose2d blueTrenchTop = new Pose2d(new Translation2d(blueTrenchBottom.getMeasureX(), fieldWidth.minus(blueTrenchBottom.getMeasureY())), Rotation2d.k180deg); //! Bad, use .getMeasureX() / .getMeasureY() instead of Inches.of(...)

    // Red side
    public static final Pose2d redHub = new Pose2d(new Translation2d(fieldLength.minus(blueHub.getMeasureX()), (fieldWidth.minus(blueHub.getMeasureY()))), Rotation2d.kZero); //! Bad, see above, it is the same cause
    public static final Pose2d redTower = new Pose2d(new Translation2d(fieldLength.minus(blueTower.getMeasureX()), (fieldWidth.minus(blueTower.getMeasureY()))), Rotation2d.k180deg);
    public static final Pose2d redDepot = new Pose2d(new Translation2d(fieldLength.minus(blueDepot.getMeasureX()), (fieldWidth.minus(blueDepot.getMeasureY()))), Rotation2d.k180deg);
    public static final Pose2d redOutpost = new Pose2d(new Translation2d(fieldLength.minus(blueOutpost.getMeasureX()), (fieldWidth.minus(blueOutpost.getMeasureY()))), Rotation2d.k180deg);
    public static final Pose2d redTrenchTop = new Pose2d(new Translation2d(fieldLength.minus(blueTrenchBottom.getMeasureX()), (fieldWidth.minus(blueTrenchBottom.getMeasureY()))), Rotation2d.kZero);
    public static final Pose2d redTrenchBottom = new Pose2d(new Translation2d(fieldLength.minus(blueTrenchTop.getMeasureX()), (fieldWidth.minus(blueTrenchTop.getMeasureY()))), Rotation2d.kZero);




}