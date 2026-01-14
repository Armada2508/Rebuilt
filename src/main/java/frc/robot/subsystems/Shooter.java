package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterK;

public class Shooter extends SubsystemBase{
    
    private final TalonFX talonShooter = new TalonFX(ShooterK.talonID);

    public Shooter(){
    }

    public void configTalons(){
    }

    public Command


}
