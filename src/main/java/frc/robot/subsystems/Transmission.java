package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.TransmissionK;
import frc.robot.lib.util.Util;

public class Transmission {
     private final TalonFX conveyor = new TalonFX(TransmissionK.talonID);

     public Transmission() {
        configTalons();
     }

     private void configTalons() {
        Util.factoryReset(conveyor);

        conveyor.getConfigurator().apply(TransmissionK.conveyorCurrentConfigs);

     }
    public void spinConveyer() { 
        conveyor.setControl(new VoltageOut(TransmissionK.spinConveyorVoltage)); //! find value

    }
    public void stop() {
        conveyor.setControl(new NeutralOut());
    }

}
