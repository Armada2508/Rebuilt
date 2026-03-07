package frc.robot.commands;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.net.SocketTimeoutException;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.Field;
import frc.robot.subsystems.shooting.Shooter;
import frc.robot.subsystems.shooting.Superstructure;

public class Routines {
    public static Command spinRoller(Intake intake) {
        return new RepeatCommand(intake.spinRoller())
        .withName("Spin Roller");
    }

    public static Command stopRoller(Intake intake) {
        return intake.stopRoller()
        .withName("Stop Roller");
    }

    public static Command intake(Intake intake) {
        return intake.extend()
        .andThen(new RepeatCommand(intake.spinRoller()))
        .withName("Intake");
    }

    public static Command stopIntake(Intake intake) {
        return intake.stopRoller()
        .andThen(intake.retract())
        .withName("Stop Intake");
    }

    public static Command stopArm(Intake intake) {
        return intake.stopArm()
        .withName("Stop Arm");
    }

    public static Command zeroEncoder(Intake intake) {
        // return runOnce(() -> intake.zeroExtender());
        return Commands.runOnce((() -> intake.zeroExtender()));
    }

    public static Command zeroGyro(Swerve swerve) {
        return swerve.commandZeroGyro();
    }
    // public static Command shoot(Indexer indexer, Shooter shooter) {
    //     return new RepeatCommand(indexer.indexCommand())
    //     .alongWith(new RepeatCommand(shooter.shootFuel()))
    //     .withName("Shoot");
    // }


    public static Command index(Indexer indexer) {
        return new RepeatCommand(indexer.indexCommand());
    }

    public static Command stopIndexer(Indexer indexer) {
        return indexer.stopCommand();
    }

    public static Command shoot(Shooter shooter, Indexer indexer) {
        // return shooter.setHoodAngle().andThen( 
        
        return shooter.shootFuel()
        .alongWith(
            Commands.waitSeconds(1).andThen(            
                indexer.indexCommand())

                ).withName("Shoot shooter");
    }

    public static Command stopShooter(Shooter shooter, Indexer indexer) {
        return shooter.stop().andThen(stopIndexer(indexer))
        .withName("Stop shooter");
    }

    public static Command setHoodAngle(Shooter shooter) {
        System.out.println("set hood angle run");
        // System.out.println("Hood Angle Units: " + targetAngle.get().unit());
        return shooter.setHoodAngle();
    }

    // public static Command stopShooter(Shooter shooter) {
    //     return Commands.runOnce(() -> shooter.stop(), shooter);
    // }

    // public static Command stowHood(Shooter shooter) {
    //     return shooter.stow().withName("Stow Hood");
    // }

    // Superstructure

    // public static Command scoreFuelHub(Superstructure superstructure, Indexer indexer) {
    //     return new RepeatCommand(indexer.indexCommand())
    //     .alongWith(new RepeatCommand(superstructure.score()))
    //     .withName("Score fuel");
    // }

    // public static Command passFuel(Superstructure superstructure, Indexer indexer) {
    //     return new RepeatCommand(indexer.indexCommand())
    //     .alongWith(new RepeatCommand(superstructure.pass()))
    //     .withName("Pass fuel");
    // }

    public static Command alignToHub(Swerve swerve) {
            return swerve.alignToPosePID(
                 () -> new Pose2d(
                    swerve.getPose().getX(), 
                    swerve.getPose().getY(),
                    Rotation2d.fromRadians(swerve.getPose().getTranslation().minus(Field.getAllianceHub().getTranslation()).getAngle().getRadians()).plus(Rotation2d.k180deg)
                )
            );
    }

    public static Command alignToPassPoint(Swerve swerve) {
        // Field.getClosestPassPoint(swerve.getPose()).getTranslation();
        return swerve.alignToPosePID(
            () -> new Pose2d(
                swerve.getPose().getX(),
                swerve.getPose().getY(),
                Rotation2d.fromRadians(swerve.getPose().getTranslation().minus(Field.getClosestPassPoint(swerve.getPose()).getTranslation()).getAngle().getRadians()).plus(Rotation2d.k180deg)
            )
        );
    }

    public static Command spinRollerRoutine(Intake intake) {
        return new RepeatCommand(intake.spinRoller())
        .withName("Spin Roller");
    }

    public static Command stopRollerRoutine(Intake intake) {
        return intake.stopRoller();
    }

}
