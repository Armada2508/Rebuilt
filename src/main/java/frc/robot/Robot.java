// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  Field2d field = new Field2d();

  public Robot() {
    SmartDashboard.putData("Arena: ", field);
   field.getObject("Blue Hub").setPose(Field.blueHub);
   field.getObject("Blue Tower").setPose(Field.blueTower);
   field.getObject("Blue Outpost").setPose(Field.blueOutpost);
   field.getObject("Blue Depot").setPose(Field.blueDepot);
    field.getObject("Blue Trench Top").setPose(Field.blueTrenchTop);
   field.getObject("Blue Trench Bottom").setPose(Field.blueTrenchBottom);

   field.getObject("Red Hub").setPose(Field.redHub);
    field.getObject("Red Tower").setPose(Field.redTower);
   field.getObject("Red Outpost").setPose(Field.redOutpost);
   field.getObject("Red Depot").setPose(Field.redDepot);
   field.getObject("Red Trench Top").setPose(Field.redTrenchTop);
   field.getObject("Red Trench Bottom").setPose(Field.redTrenchBottom);
   


  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
