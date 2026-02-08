// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.reduxrobotics.canand.CanandEventLoop;

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
import frc.robot.lib.util.DriveUtil;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

@Logged
public class Robot extends TimedRobot {
    private final XboxController xboxController = new XboxController(ControllerK.xboxPort);

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
