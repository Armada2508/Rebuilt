// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.ControllerK;
import frc.robot.Constants.DriveK;
import frc.robot.Constants.VisionK;
import frc.robot.commands.Autos;
import frc.robot.commands.Routines;
import frc.robot.lib.logging.LogUtil;
import frc.robot.lib.util.DriveUtil;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.shooting.Shooter;
import frc.robot.subsystems.shooting.Superstructure;
import frc.robot.subsystems.shooting.Turret;

@Logged
public class Robot extends TimedRobot {
    private final CommandXboxController xboxController = new CommandXboxController(ControllerK.xboxPort);
    // private Alliance activeHub;

    private Field2d field = new Field2d();
    @Logged(name = "Intake")
    private Intake intake = new Intake();
    // @Logged(name = "Shooter")
    private Shooter shooter = new Shooter();
    // @Logged(name = "Turret")
    private Turret turret = new Turret(); // For logging
    @Logged(name = "Superstructure")
    private Superstructure superstructure = new Superstructure(shooter, turret);
    @Logged(name = "Indexer")
    private Indexer indexer = new Indexer();

     private final SendableChooser<Command> autoChooser;

    @Logged(name = "Vision")
    private Vision vision = new Vision();
    @Logged(name = "Swerve")
    private final Swerve swerve = new Swerve(vision::getVisionResults, vision , () -> 
        Math.abs(xboxController.getLeftX()) > ControllerK.overrideThreshold
        || Math.abs(xboxController.getLeftY()) > ControllerK.overrideThreshold
        || Math.abs(xboxController.getRightX()) > ControllerK.overrideThreshold);

    public Robot() {
        DataLog dataLog = DataLogManager.getLog();
        DriverStation.silenceJoystickConnectionWarning(true);
        LogUtil.logDriverStation(this); // Network Tables
        LogUtil.logCommandInterrupts(dataLog); // Network Tables & DataLog
        DriverStation.startDataLog(dataLog); // DataLog
        Epilogue.bind(this);
        swerve.setDefaultCommand(teleopDriveCommand());
        configureBindings();
        logFieldConstants();
        autoChooser = Autos.initPathPlanner(shooter, intake, indexer);
    
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
        field.getObject("Blue Zone Corner 1").setPose(Field.blueZoneCorner1);
        field.getObject("Blue Zone Corner 2").setPose(Field.blueZoneCorner2);

        field.getObject("Red Hub").setPose(Field.redHub);
        field.getObject("Red Tower").setPose(Field.redTower);
        field.getObject("Red Outpost").setPose(Field.redOutpost);
        field.getObject("Red Depot").setPose(Field.redDepot);
        field.getObject("Red Trench Left").setPose(Field.redTrenchLeft);
        field.getObject("Red Trench Right").setPose(Field.redTrenchRight);
        field.getObject("Pass Target Red High").setPose(Field.passTargetRedHigh);
        field.getObject("Pass Target Red Low").setPose(Field.passTargetRedLow);
        field.getObject("Red Zone Corner 1").setPose(Field.redZoneCorner1);
        field.getObject("Red Zone Corner 2").setPose(Field.redZoneCorner2);
    }
  
    @Override
    public void robotInit() {
        CanandEventLoop.getInstance();
        //^ This might not be needed, depends on if we need to initialize canandmag encoders in Swerve.java
    }

        @Override
    public void autonomousInit() {
        var selected = autoChooser.getSelected();
        if (selected instanceof PathPlannerAuto auto) {
            if (!swerve.initializedOdometryFromVision()) {
                var pose = auto.getStartingPose();
                if (onRedAlliance()) {
                    pose = FlippingUtil.flipFieldPose(pose);
                }
                swerve.resetOdometry(pose);
            }
        }
        selected.schedule();
    }
  
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // SmartDashboard.putData("Time left in current phase", HubShiftUtil.getOfficialShiftInfo().remainingTime());
        SmartDashboard.putNumber("Time left in current phase", HubShiftUtil.getOfficialShiftInfo().remainingTime());
        SmartDashboard.putBoolean("Is Hub Active", HubShiftUtil.getShiftedShiftInfo().active());
        SmartDashboard.putString("Current Phase", HubShiftUtil.getOfficialShiftInfo().currentShift().toString());
    }

    @Override
    public void teleopPeriodic() {
        // activeHub = getActiveHub();
        // SmartDashboard.putBoolean("Alliance Hub Active", isHubActive()); //! THIS WILL NOT LOG IF THE ROBOT IS DISCONNECTED IN SIM
    }

//     public Alliance getActiveHub() {
//             String gameData = DriverStation.getGameSpecificMessage();
//             double matchTime = DriverStation.getMatchTime();

//             if (getMatchPhase(matchTime).equals("Transition") || getMatchPhase(matchTime).equals("End Game")) return DriverStation.getAlliance().get();

//             if (gameData.length() > 0) {
//                 switch (gameData.charAt(0)) {
//                     case 'B':
//                         if (getMatchPhase(matchTime).equals("Shift One") || getMatchPhase(matchTime).equals("Shift Three")) return Alliance.Red;
//                         else return Alliance.Blue;
//                     case 'R':
//                         if (getMatchPhase(matchTime).equals("Shift One") || getMatchPhase(matchTime).equals("Shift Three")) return Alliance.Blue;
//                         else return Alliance.Red;
//                     default:
//                         System.out.println("Corrupt Data");
//                         return DriverStation.getAlliance().get();
//                 }
//             }
//             else {
//                 System.out.println("No Data Recieved Yet");
//                 return DriverStation.getAlliance().get();
//             }
//     }

//     public boolean isHubActive() {
//         if (getActiveHub().equals(DriverStation.getAlliance().get())) return true;
//         return false;
//     }

// public static String getMatchPhase(double matchTime) {
//     if (matchTime >= 130) {
//         return "Transition";
//     }
//     else if (129 > matchTime && matchTime >= 105) {
//         return "Shift One";
//     }
//         else if (104 > matchTime && matchTime >= 80) {
//         return "Shift Two";
//     }
//         else if (79 > matchTime && matchTime >= 55) {
//         return "Shift Three";
//     }
//         else if (54 > matchTime && matchTime >= 30) {
//         return "Shift Four";
//     }
//     else if (matchTime < 30) {
//         return "End Game";
//     }
//     else return "How did we get here";
// }

    public void configureBindings() {
        //~ Intake Routines
        Command intakeCommand = Routines.intake(intake);
        Command stopIntake = Routines.stopIntake(intake);
        Command spinRoller = Routines.spinRollerRoutine(intake);
        Command stopRoller = Routines.stopRollerRoutine(intake);
        Command extend = Routines.intake(intake); //! Create
        Command retract = Routines.stopIntake(intake); //! Create
        Command stopArm = Routines.stopArm(intake);
        Command zeroEncoder = Routines.zeroEncoder(intake);
        // Command setHoodAngle = Routines.setHoodAngle(shooter);

        // Command setHoodAngle = Routines.setHoodAngle(shooter);
        
        // Command hoodTenDegrees = Routines.setHoodAngle(shooter, Degrees.of(20));
        // Command hoodTwentyDegrees = Routines.setHoodAngle(shooter, () -> Degrees.of(20));

        //~ Shooter Routines
        Command shoot = Routines.shoot(shooter, indexer);
        Command stopShooter = Routines.stopShooter(shooter, indexer);
        // Command stowRoutine = Routines.stowHood(shooter); 

        //~ Indexer Routines
        Command index = Routines.index(indexer);
        Command stopIndex = Routines.stopIndexer(indexer);

        //! AVOID BINDING TO 'Y'
        Command zeroGyro = Routines.zeroGyro(swerve);
        //~ Debugging / Simulation
        // xboxController.povDown().whileTrue(swerve.characterizeDriveWheelDiameter());
        // xboxController.a().whileTrue(swerve.faceWheelsForward());
        // xboxController.b().whileTrue(swerve.setDriveVoltage(Volts.of(1)));
        // xboxController.b().onTrue(Commands.runOnce(() -> swerve.resetOdometry(new Pose2d(Meters.of(2), Meters.of(2), Rotation2d.kZero)), swerve)); // For simulation        

        //~ Shooting & Indexing
        // xboxController.a().whileTrue(index)
        //  .onFalse(stopIndex);

        xboxController.rightTrigger().whileTrue(shoot)
        .onFalse(stopShooter);

        xboxController.a().onTrue(Routines.setHoodAngle(shooter));
        // xboxController.povUp().onTrue(Commands.print("Button pressed"));

        // xboxController.povDown().onTrue(hoodTwentyDegrees);


        //~ Intaking
        //xboxController.leftTrigger().whileTrue(spinRoller) 
        //.onFalse(stopRoller);
        xboxController.rightBumper().whileTrue(extend)
        .onFalse(stopArm);
        
        xboxController.leftBumper().onTrue(retract)
        .onFalse(stopArm);

        xboxController.leftTrigger().whileTrue(intakeCommand)
        .onFalse(stopIntake);

        xboxController.x().onTrue(spinRoller)
        .onFalse(stopRoller);

        // xboxController.a().onTrue(setHoodAngle);

        // xboxController.y().onTrue(zeroEncoder);

        //~ Alignment
        // xboxController.a().onTrue(Routines.alignToHub(swerve));
        // xboxController.b().onTrue(Routines.alignToPassPoint(swerve));
        // xboxController.povUp().onTrue(zeroGyro);
        // Superstructure
        // Command scoreRoutine = Routines.scoreFuelHub(superstructure, indexer);
        // Command passRoutine = Routines.passFuel(superstructure, indexer);

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

    @Logged(name = "Robot to Front Camera")
    public Transform3d getRobotToCameraTransform() {
        return VisionK.robotToFrontCamera;
    }

}
