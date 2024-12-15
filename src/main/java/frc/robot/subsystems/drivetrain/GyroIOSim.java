package frc.robot.subsystems.drivetrain;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;
    
    public GyroIOSim(GyroSimulation gyroSim) {
        this.gyroSimulation = gyroSim;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.angle = gyroSimulation.getGyroReading();
        inputs.rate = gyroSimulation.getMeasuredAngularVelocity().in(Units.RadiansPerSecond);
    }

    @Override
    public void calibrate() {
        gyroSimulation.setRotation(new Rotation2d(0));
    }
}
