package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    public Turret() {

    }

    private void configTalons() {

    }

    private void configMotionMagic() {

    }

    public void setAngle(Angle targetAngle) {

    }

    public Command setAngleCommand(Angle targetAngle) {
        //! Verify
        return runOnce(() -> setAngle(targetAngle));

    }

    public Angle getAngle() {
        //! Create
        return null; 
    }


}
