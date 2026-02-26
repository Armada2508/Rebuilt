package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooting.Shooter;

public class Autos {
    
    private static SendableChooser<Command> autoChooser;

    public Autos(){}

    public static SendableChooser<Command> initPathPlanner(Shooter shooter){

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("Shoot Fuel", shooter.shootFuel());

        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
    }


    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

