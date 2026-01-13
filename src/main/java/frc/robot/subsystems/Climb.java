package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbK;

public class Climb extends SubsystemBase{
    
    private final TalonFX talon = new TalonFX(ClimbK.talonID);
}
