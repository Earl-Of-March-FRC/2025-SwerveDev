package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class GyroIOADXRS450 implements GyroIO {
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final Supplier<Rotation2d> angleSupplier = () -> gyro.getRotation2d();
    private final Supplier<Double> rateSupplier = () -> gyro.getRate();

    public GyroIOADXRS450() {
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.angle = angleSupplier.get();
        inputs.rate = rateSupplier.get();
    }

    @Override
    public void calibrate() {
        gyro.calibrate();
        gyro.reset();
    }
}
