package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface ATVisionIO {
    @AutoLog
    public static class ATVisionIOInputs {
        public boolean[] connected;
        public double latencySeconds;
        public int targetsCount;
        public int[] targetIDs;
        public Transform3d[] cameraToTargets;
        public Pose3d[] robotPoses;
        public double[] timestamp;
    }

    public default void updateInputs(ATVisionIOInputs inputs) {
    }

    public default void setReferencePose(Pose2d prevEstimatedPose) {
    }
}
