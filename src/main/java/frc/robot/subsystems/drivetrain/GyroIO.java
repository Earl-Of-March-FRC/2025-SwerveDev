package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d angle = new Rotation2d();
        public double rate = 0;
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default void calibrate() {
    }
}
