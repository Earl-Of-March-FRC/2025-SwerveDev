package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

public class ATVisionIOSim implements ATVisionIO {
    private final VisionSystemSim visionSystemSim;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;
    private Pose2d prevEstimatedPose = new Pose2d();
    private Supplier<Pose2d> trueRobotPoseSup;

    public ATVisionIOSim(
        List<Pair<String, SimCameraProperties>> cameraProperties,
        List<Transform3d> robotToCamera,
        AprilTagFieldLayout aprilTagFieldLayout,
        Supplier<Pose2d> trueRobotPoseSupplier, Pose2d initialPose) {
        this(cameraProperties, robotToCamera, aprilTagFieldLayout, trueRobotPoseSupplier);
        setReferencePose(initialPose);
    }

    public ATVisionIOSim(
            List<Pair<String, SimCameraProperties>> cameraProperties,
            List<Transform3d> robotToCamera,
            AprilTagFieldLayout aprilTagFieldLayout,
            Supplier<Pose2d> trueRobotPoseSupplier) {
        if (cameraProperties.size() > 16) throw new IllegalArgumentException("Max 16 cameras supported");
        if (cameraProperties.size() != robotToCamera.size()) throw new IllegalArgumentException("cameraProperties and robotToCamera must be the same size");

        visionSystemSim = new VisionSystemSim("main");
        visionSystemSim.addAprilTags(aprilTagFieldLayout);

        PhotonCameraSim[] cameraSims = new PhotonCameraSim[cameraProperties.size()];
        cameras = new PhotonCamera[cameraProperties.size()];
        for (int i = 0; i < cameraProperties.size(); i++) {
            cameraSims[i] = new PhotonCameraSim(new PhotonCamera(cameraProperties.get(i).getFirst()), cameraProperties.get(i).getSecond());
            cameraSims[i].enableRawStream(true);
            cameraSims[i].enableProcessedStream(true);
            visionSystemSim.addCamera(cameraSims[i], robotToCamera.get(i));
            cameras[i] = cameraSims[i].getCamera();
        }

        poseEstimators = new PhotonPoseEstimator[cameraProperties.size()];
        for (int i = 0; i < cameraProperties.size(); i++) {
            poseEstimators[i] = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamera.get(i));
        }

        trueRobotPoseSup = trueRobotPoseSupplier;
    }

    @Override
    public void updateInputs(ATVisionIOInputs inputs) {
        visionSystemSim.update(trueRobotPoseSup.get());
        SmartDashboard.putData(visionSystemSim.getDebugField());

        for (int i = 0; i < cameras.length; i++) {
            poseEstimators[i].setReferencePose(prevEstimatedPose);
        }

        inputs.connected = new boolean[cameras.length];

        List<Integer> targetIDs = new ArrayList<>();
        List<Transform3d> cameraToTargets = new ArrayList<>();
        List<Pose3d> robotPoses = new ArrayList<>();
        List<Double> timestamps = new ArrayList<>();

        for (int i = 0; i < cameras.length; i++) {
            inputs.connected[i] = true;
            PhotonCamera camera = cameras[i];
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            PhotonPipelineResult result;
            if (results.isEmpty()) {
                continue;
            } else {
                result = results.get(0);
            }
            if (result.hasTargets()) {
                List<PhotonTrackedTarget> targets = result.getTargets();
                for (PhotonTrackedTarget target : targets) {
                    if (target.getPoseAmbiguity() <= VisionConstants.kATAmbiguityThreshold) {
                        targetIDs.add(target.getFiducialId());
                        cameraToTargets.add(target.getBestCameraToTarget());
                    }
                }
            }

            Optional<EstimatedRobotPose> pose = poseEstimators[i].update(result);
            if (pose.isPresent()) {
                robotPoses.add(pose.get().estimatedPose);
                timestamps.add(pose.get().timestampSeconds);
                inputs.targetsCount = targetIDs.size();
                inputs.targetIDs = targetIDs.stream().mapToInt(a -> a).toArray();
                inputs.cameraToTargets = cameraToTargets.toArray(new Transform3d[0]);
                inputs.robotPoses = robotPoses.toArray(new Pose3d[0]);
                inputs.timestamp = timestamps.stream().mapToDouble(a -> a).toArray();
            }
        }

        
    }

    @Override
    public void setReferencePose(Pose2d prevEstimatedPose) {
        this.prevEstimatedPose = prevEstimatedPose;
    }
}
