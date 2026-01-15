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
    public static final Pose2d blueHub = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(158.84)), Rotation2d.kZero); 
    public static final Pose2d blueTower = new Pose2d(new Translation2d(Inches.of(41.56), Inches.of(147.47)), Rotation2d.kZero);
    public static final Pose2d blueDepot = new Pose2d(new Translation2d(Inches.of(15.5), Inches.of(241.75)), Rotation2d.kZero); //! This is off
    public static final Pose2d blueOutpost = new Pose2d(new Translation2d(Inches.of(0), Inches.of(26.22)), Rotation2d.kZero);
    public static final Pose2d blueTrenchBottom = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(24.92)), Rotation2d.kZero);
    public static final Pose2d blueTrenchTop = new Pose2d(new Translation2d(Inches.of(blueTrenchBottom.getX()), fieldWidth.minus(Inches.of(blueTrenchBottom.getY()))), Rotation2d.kZero); //! Bad, use .getMeasureX() / .getMeasureY() instead of Inches.of(...)

    // Red side
    public static final Pose2d redHub = new Pose2d(new Translation2d(fieldLength.minus(Inches.of(blueHub.getX())), (fieldWidth.minus(Inches.of(blueHub.getY())))), Rotation2d.kZero); //! Bad, see above, it is the same cause





}