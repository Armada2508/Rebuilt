package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.Field;
import frc.robot.subsystems.shooting.Shooter;
import frc.robot.subsystems.shooting.Maps;
import frc.robot.subsystems.shooting.Superstructure;

public class Routines {
    public static Command intake(Intake intake) {
        return intake.extend()
        .andThen(new RepeatCommand(intake.spinRoller()))
        .withName("Intake");
    }

    public static Command stopIntake(Intake intake) {
        return intake.retract().andThen(intake.stopRoller()).withName("Stop Intake");
    }

    // public static Command shoot(Indexer indexer, Shooter shooter) {
    //     return new RepeatCommand(indexer.indexCommand())
    //     .alongWith(new RepeatCommand(shooter.shootFuel()))
    //     .withName("Shoot");
    // }

    public static Command shoot(Indexer indexer) {
        return new RepeatCommand(indexer.indexCommand())
        .withName("Shoot");
    }

    public static Command stopIndexer(Indexer indexer) {
        return indexer.stopCommand();
    }

    public static Command stopShooter(Shooter shooter) {
        return shooter.stop().withName("Stop Shooter");
    }

    public static Command stowHood(Shooter shooter) {
        return shooter.stow().withName("Stow Hood");
    }

    // Superstructure

    public static Command scoreFuelHub(Superstructure superstructure, Indexer indexer) {
        return new RepeatCommand(indexer.indexCommand())
        .alongWith(new RepeatCommand(superstructure.score()))
        .withName("Score fuel");
    }

    public static Command passFuel(Superstructure superstructure, Indexer indexer) {
        return new RepeatCommand(indexer.indexCommand())
        .alongWith(new RepeatCommand(superstructure.pass()))
        .withName("Pass fuel");
    }

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
