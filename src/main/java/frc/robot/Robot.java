// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.commands.Routines;
import frc.robot.lib.util.DriveUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

@Logged
public class Robot extends TimedRobot {
    private final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);

    @Logged
    private Vision vision = new Vision();
    @Logged(name = "Swerve")
    private final Swerve swerve = new Swerve(vision::getVisionResults, () -> 
        Math.abs(xboxController.getLeftX()) > ControllerK.overrideThreshold
        || Math.abs(xboxController.getLeftY()) > ControllerK.overrideThreshold
        || Math.abs(xboxController.getRightX()) > ControllerK.overrideThreshold);

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        Epilogue.bind(this);
        swerve.setDefaultCommand(teleopDriveCommand());
        configureBindings();
        logFieldConstants();
    }

    public void logFieldConstants() {
        SmartDashboard.putData("Arena: ", field);
        field.getObject("Blue Hub").setPose(Field.blueHub);
        field.getObject("Blue Tower").setPose(Field.blueTower);
        field.getObject("Blue Outpost").setPose(Field.blueOutpost);
        field.getObject("Blue Depot").setPose(Field.blueDepot);
        field.getObject("Blue Trench Left").setPose(Field.blueTrenchLeft);
        field.getObject("Blue Trench Right").setPose(Field.blueTrenchRight);
        field.getObject("Pass Target Blue High").setPose(Field.passTargetBlueHigh);
        field.getObject("Pass Target Blue Low").setPose(Field.passTargetBlueLow);

        field.getObject("Red Hub").setPose(Field.redHub);
        field.getObject("Red Tower").setPose(Field.redTower);
        field.getObject("Red Outpost").setPose(Field.redOutpost);
        field.getObject("Red Depot").setPose(Field.redDepot);
        field.getObject("Red Trench Left").setPose(Field.redTrenchLeft);
        field.getObject("Red Trench Right").setPose(Field.redTrenchRight);
        field.getObject("Pass Target Red High").setPose(Field.passTargetRedHigh);
        field.getObject("Pass Target Red Low").setPose(Field.passTargetRedLow);
    }
  
    @Override
    public void robotInit() {
        CanandEventLoop.getInstance();
        //^ This might not be needed, depends on if we need to initialize canandmag encoders in Swerve.java
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    public void configureBindings() {
        xboxController.povDown().whileTrue(swerve.characterizeDriveWheelDiameter());
        xboxController.a().whileTrue(swerve.faceWheelsForward());
        xboxController.b().whileTrue(swerve.setDriveVoltage(Volts.of(1)));

        xboxController.y().onTrue(Routines.alignToHubPID(swerve));
    
    }

    public Command teleopDriveCommand() {
        return swerve.driveCommand(
            () -> {
                double val = MathUtil.applyDeadband(-xboxController.getLeftY(), ControllerK.leftJoystickDeadband);
                val = DriveUtil.powKeepSign(val, DriveK.exponentialControl);
                val *= DriveK.driveSpeedModifier;
                return val;
            }, 
            () -> {
                double val = MathUtil.applyDeadband(-xboxController.getLeftX(), ControllerK.leftJoystickDeadband);
                val = DriveUtil.powKeepSign(val, DriveK.exponentialControl);
                val *= DriveK.driveSpeedModifier;
                return val; 
            },  
            () -> {
                double val = MathUtil.applyDeadband(-xboxController.getRightX(), ControllerK.rightJoystickDeadband);
                val = DriveUtil.powKeepSign(val, DriveK.exponentialControl);
                val *= DriveK.rotationSpeedModifier;
                return val; 
            },
            true, true
        ).withName("Swerve Drive Field Oriented");
    }

    public static boolean onRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }


}
