package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.TransmissionK;
import frc.robot.lib.util.Util;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Transmission {

    private final TalonSRX conveyor = new TalonSRX(TransmissionK.talonID);

    public Transmission() {
        configTalons();
    }

    private void configTalons() {
        conveyor.configFactoryDefault();

        
        conveyor.setNeutralMode(NeutralMode.Brake);
        conveyor.configPeakOutputForward(1.0);
        conveyor.configPeakOutputReverse(-1.0);
    }

    public void spinConveyer() {
        
        conveyor.set(ControlMode.PercentOutput, TransmissionK.spinConveyorPercent);
    }

    public void stop() {
        conveyor.set(ControlMode.PercentOutput, 0.0);
    }
}

