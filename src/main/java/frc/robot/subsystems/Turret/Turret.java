package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase {


        private TurretIOReal io;
    
        public Turret(TurretIOReal io) {
            this.io = io;
    }


    public Command runPercent(double percent) {
        return runEnd(() -> io.setPower(percent * 12.0), () -> io.setPower(0.0));
    }
}