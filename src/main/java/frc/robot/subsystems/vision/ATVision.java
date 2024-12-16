package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ATVision extends SubsystemBase {
    private final ATVisionIO io;
    private final ATVisionIOInputsAutoLogged inputs = new ATVisionIOInputsAutoLogged();
    
    public ATVision(ATVisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision/AprilTags", inputs);
    }
    
    public void setReferencePose(Pose2d prevEstimatedPose) {
        io.setReferencePose(prevEstimatedPose);
    }

    public Optional<Pair<Pose3d, Double>> getBestFieldToRobot() {
        Optional<Pose3d> bestFieldToRobot = Optional.empty();
        if (inputs.robotPoses != null && inputs.robotPoses.length > 0) {
            int validValues = 0;
            double sumX = 0, sumY = 0, sumZ = 0, sumRoll = 0, sumPitch = 0, sumYaw = 0;
            for (Pose3d pose : inputs.robotPoses) {
                validValues++;
                Pose3d fieldToRobot = pose;
                sumX += fieldToRobot.getTranslation().getX();
                sumY += fieldToRobot.getTranslation().getY();
                sumZ += fieldToRobot.getTranslation().getZ();
                sumRoll += fieldToRobot.getRotation().getX();
                sumPitch += fieldToRobot.getRotation().getY();
                sumYaw += fieldToRobot.getRotation().getZ();
            }
            if (validValues > 0) {
                bestFieldToRobot = Optional.of(new Pose3d(
                    new Translation3d(sumX / validValues, sumY / validValues, sumZ / validValues),
                    new Rotation3d(sumRoll / validValues, sumPitch / validValues, sumYaw / validValues)
                ));
            }
        }

        double avgTimestamp = 0;
        if (inputs.timestamp != null && inputs.timestamp.length > 0) {
            for (double timestamp : inputs.timestamp) {
                avgTimestamp += timestamp;
            }
            avgTimestamp /= inputs.timestamp.length;
        }

        if (bestFieldToRobot.isPresent()) {
            Logger.recordOutput("Vision/BestPose", bestFieldToRobot.get());
        }

        if (avgTimestamp > 0 && bestFieldToRobot.isPresent()) {
            return Optional.of(Pair.of(bestFieldToRobot.get(), avgTimestamp));
        } else {
            return Optional.empty();
        }
    }
}
