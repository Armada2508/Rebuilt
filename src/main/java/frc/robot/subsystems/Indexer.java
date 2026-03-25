package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerK;
import frc.robot.lib.util.Util;

@Logged
public class Indexer extends SubsystemBase{

    private TalonFX talon = new TalonFX(IndexerK.id);

    private SparkMax sparkLeft = new SparkMax(IndexerK.sparkLeftID, MotorType.kBrushless);
    private SparkMax sparkRight = new SparkMax(IndexerK.sparkRightID, MotorType.kBrushless);

    public Indexer() {
        configTalons();
    }

    public void configTalons() {
        Util.factoryReset(talon);
        Util.coastMode(talon);
        // talon.getConfigurator().apply(IndexerK.currentLimitConfig);
    }

    @SuppressWarnings("removal")
    public void configSparkMaxs() {
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();
    
        leftConfig.idleMode(IdleMode.kCoast);
        rightConfig.idleMode(IdleMode.kCoast);

        leftConfig.smartCurrentLimit(IndexerK.leftCurrentLimit);
        rightConfig.smartCurrentLimit(IndexerK.rightCurrentLimit);

        leftConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .warningsAlwaysOn(true)
        .faultsAlwaysOn(true);

        rightConfig.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .warningsAlwaysOn(true)
        .faultsAlwaysOn(true);

        sparkLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sparkRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void index() {
        talon.setVoltage(IndexerK.indexingVoltage.in(Volts));
        sparkLeft.setVoltage(IndexerK.leftAgitatorSpinVoltage.in(Volts));
        sparkRight.setVoltage(-IndexerK.rightAgitatorSpinVoltage.in(Volts)); //! flip?
    }

    public Command indexCommand() {
        return runOnce(() -> index()); //! Check
    }
    public Command stopCommand() {
        return runOnce(() -> stop());
    }

    public void stop() {
        // return runOnce(() -> talon.setControl(new NeutralOut()));
        talon.setControl(new NeutralOut());
        sparkLeft.stopMotor();
        sparkRight.stopMotor();
    }

    /**
     * Returns the currently running command
     * @return The command being run
     */
    @Logged(name = "Current Command")
    public String getCurrentCommandName() {
        var cmd = getCurrentCommand();
        if (cmd == null) return "None";
        return cmd.getName();
    }
}
