package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private Double angle = 0.0;
    private Double rate = 0.0;

    public GyroIOSim() {
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        angle += rate * LOOP_PERIOD_SECS;

        inputs.connected = true;
        inputs.angle = new Rotation2d(angle);
        inputs.rate = rate;
    }

    @Override
    public void calibrate() {
        rate = 0.0;
        angle = 0.0;
    }

    public void updateAngularVelocity(double angularVelocity) {
        rate = angularVelocity;
    }

    public void updateAngle(double angle) {
        this.angle = angle;
    }
}
