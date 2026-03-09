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
import frc.robot.subsystems.shooting.Shooter;

public class Autos {
    
    private Autos(){}

    public static SendableChooser<Command> initPathPlanner(Shooter shooter, Intake intake, Indexer indexer){
        FollowPathCommand.warmupCommand().schedule();
        

        NamedCommands.registerCommand("Shoot Fuel", shooter.shootFuel().alongWith(indexer.indexCommand()));

        new EventTrigger("intake fuel").onTrue(Routines.intake(intake));
        new EventTrigger("stop intaking").onTrue(Routines.stopIntake(intake));

        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        return autoChooser;
    }
}

