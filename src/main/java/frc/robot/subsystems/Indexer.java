package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerK;
import frc.robot.lib.util.Util;

@Logged
public class Indexer extends SubsystemBase{
    //! This is a TEMPORARY class for week 0 because of current architecture.
    //! This should eventually be put inside IndexerOld.java before Winona

    private TalonFX talon = new TalonFX(IndexerK.id);

    public Indexer() {
        configTalons();
    }

    public void configTalons() {
        Util.factoryReset(talon);
        Util.coastMode(talon);
        talon.getConfigurator().apply(IndexerK.currentLimitConfig);
    }

    public void index() {
        talon.setVoltage(IndexerK.indexingVoltage.in(Volts));
    }

    public Command indexCommand() {
        return runOnce(() -> index()); //! Check
    }
}
