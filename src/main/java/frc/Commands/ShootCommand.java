package frc.Commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.Shooter;

public class ShootCommand {
    public static Command AutoShoot() {
        Hopper hopper = Hopper.getInstance();
        Turret turret = Turret.getInstance();
        Shooter shooter = Shooter.getInstance();

        return new SequentialCommandGroup()
    }
}
