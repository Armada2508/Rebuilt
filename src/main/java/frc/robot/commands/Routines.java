package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Field;

public class Routines {
    public static Command alignToHubPID(Swerve swerve) {
        System.out.println("command running");
        Pose2d targetPose = new Pose2d(
                swerve.getPose().getX(), 
                swerve.getPose().getY(),
                Field.getAllianceHub().getRotation());

        System.out.println("target pose created");
        return swerve.alignToPosePID(
            () -> targetPose
        );
    }
}
