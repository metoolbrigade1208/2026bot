package frc.robot.subsystems.Turret;

import static frc.robot.subsystems.Turret.SparkUtil.tryUntilOk;
import static frc.robot.subsystems.Turret.SparkUtil.ifOk;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;


public class TurretIOReal implements TurretIO {
    public static final int motorCanId = 6;
        private static final double motorReduction = 0;
            protected final SparkMax azimuthMotor = new SparkMax(motorCanId, MotorType.kBrushless);
        private final RelativeEncoder encoder = azimuthMotor.getEncoder();
      
        @SuppressWarnings("removal")
        public TurretIOReal() {
            var config = new SparkMaxConfig();
            int currentLimit = 40;
                    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
            config
                .encoder
                .positionConversionFactor(2.0 * Math.PI / motorReduction) // Rotor Rotations -> Radians
            .velocityConversionFactor((2.0 * Math.PI) / 60.0 / motorReduction)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
  
        tryUntilOk(azimuthMotor, 5, () ->
            azimuthMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }
  
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        ifOk(azimuthMotor, encoder::getPosition, (value) -> inputs.positionRad = value);
        ifOk(azimuthMotor, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
        ifOk(
            azimuthMotor,
            new DoubleSupplier[] {azimuthMotor::getAppliedOutput, azimuthMotor::getBusVoltage},
            (values) -> inputs.appliedVolts = values[0] * values[1]);
        ifOk(azimuthMotor, azimuthMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }
  
    @Override
    public void setPower(double power) {
        azimuthMotor.set(power);
    }
}
