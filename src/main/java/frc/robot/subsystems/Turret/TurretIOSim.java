package frc.robot.subsystems.Turret;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class TurretIOSim extends TurretIOReal {
    // DC simulation motors
    protected static DCMotor flywheelSimMotor = DCMotor.getNeoVortex(2);
    protected static DCMotor azimuthSimMotor = DCMotor.getNeoVortex(1);
    protected static DCMotor hoodSimMotor = DCMotor.getNeoVortex(1);

    // Spark simulation objects
    //protected SparkFlexSim topFlywheelMotorSim = new SparkFlexSim(topFlywheelMotor, flywheelSimMotor);
    //protected SparkFlexSim bottomFlywheelMotorSim = new SparkFlexSim(bottomFlywheelMotor, flywheelSimMotor);
    protected SparkMaxSim azimuthMotorSim = new SparkMaxSim(azimuthMotor, azimuthSimMotor);
   // protected SparkFlexSim hoodMotorSim = new SparkFlexSim(hoodMotor, hoodSimMotor);

    // Spark simulation sensors
    protected SparkAbsoluteEncoderSim azimuthEncoderSim = azimuthMotorSim.getAbsoluteEncoderSim();

    public TurretIOSim() {
        super();
    }
  
    public void updateInputs(TurretIOInputs inputs) {
        // Outputs should already be set
        // TODO: Sim

        super.updateInputs(inputs);
    }
}