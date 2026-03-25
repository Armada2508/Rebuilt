package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooting.Shooter;

public class Autos {
    
    private Autos(){}

    public static SendableChooser<Command> initPathPlanner(Swerve swerve, Shooter shooter, Intake intake, Indexer indexer){
        FollowPathCommand.warmupCommand().schedule();
        

        NamedCommands.registerCommand("Shoot Fuel", Routines.shootInterpolatedRpm(swerve, shooter));

        NamedCommands.registerCommand("Stop Intaking", Routines.stopIntake(intake));

        new EventTrigger("intake fuel").onTrue(Routines.intake(intake));
        new EventTrigger("stop intaking").onTrue(Routines.stopIntake(intake));
        new EventTrigger("shoot fuel").onTrue(Routines.shootInterpolatedRpm(swerve, shooter));

        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}

