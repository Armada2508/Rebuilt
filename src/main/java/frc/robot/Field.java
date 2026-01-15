package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Field {
    // X is left to right, Y is top to bottom

    public static final Pose2d origin = Pose2d.kZero;

    // Field
    public static final Distance fieldLength = Inches.of(651.22);
    public static final Distance fieldWidth = Inches.of(317.69);
    public static final Pose2d fieldCenter = new Pose2d(fieldLength.div(2), fieldWidth.div(2), Rotation2d.kZero);

    // Blue side
    public static final Pose2d BlueHub = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(158.84)), Rotation2d.kZero);
    public static final Pose2d BlueTower = new Pose2d(new Translation2d(Inches.of(41.56), Inches.of(147.47)), Rotation2d.kZero);
    public static final Pose2d BlueDepot = new Pose2d(new Translation2d(Inches.of(15.5), Inches.of(241.75)), Rotation2d.kZero);
    public static final Pose2d BlueOutpost = new Pose2d(new Translation2d(Inches.of(0), Inches.of(26.22)), Rotation2d.kZero);
    public static final Pose2d BlueTrenchBottom = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(24.92)), Rotation2d.kZero);
    public static final Pose2d BlueTrenchTop = new Pose2d(new Translation2d(Inches.of(BlueTrenchBottom.getX()), fieldWidth.minus(Inches.of(BlueTrenchBottom.getY()))), Rotation2d.kZero);

    // Red side

    public static final Pose2d RedHub = new Pose2d(new Translation2d(fieldLength.minus(Inches.of(BlueHub.getX())), (fieldWidth.minus(Inches.of(BlueHub.getY())))), Rotation2d.kZero);




}