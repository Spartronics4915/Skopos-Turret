package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.VisionConstants.VisionState.GLOBAL;

import frc.robot.Constants.VisionConstants.CameraConfig;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase {

    private Map<String, PhotonCamera> cameras = new HashMap<>();
    private Map<String, Matrix<N3, N1>> stdDevs = new HashMap<>();
    private Map<String, PhotonPoseEstimator> estimators = new HashMap<>();
    
    private VisionSystemSim visionSystemSim;
    private VisionState visionState = GLOBAL;
    private boolean isSimulation;
    private boolean isDebugging;
    private Supplier<Pose2d> poseSupplier;
    
    private int trackedTagId;
    private List<Pose3d> visibleTagPoses = new ArrayList<>();
    
    private PhotonCamera currentCamera;
    private String currentCameraName;
    private PhotonPoseEstimator currentEstimator;
    private EstimatedRobotPose currentEstimatedRobotPose;
    private Pose3d currentRobotPose;
    
    private int amountOfTagsUsed;
    private double currentDistanceToTarget;
    private double trustedDistance;
    private double totalDistance;
    private double avgDistance;
    private int amountOfTrustedTags;
    private int closestTagId;

    public static double visionPoseTimestamp;
    private Supplier<Pose2d> pastVisionPoseSupplier;

    private final VisionConsumer visionConsumer;
    private double translationStdDevs;
    private double rotationStdDevs;
    private Matrix<N3, N1> currentStdDevs;

    StructPublisher<Pose2d> rawVisionPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Raw Vision Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> compensatedVisionPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Compensated Vision Pose", Pose2d.struct).publish();
    StructArrayPublisher<Pose3d> trackedTagsPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Tracked Tags", Pose3d.struct).publish();
        
    public VisionSubsystem(VisionConsumer visionConsumer, Supplier<Pose2d> poseSupplier, Supplier<Pose2d> pastVisionPoseSupplier) {
        this.visionConsumer = visionConsumer;
        this.poseSupplier = poseSupplier;
        this.pastVisionPoseSupplier = pastVisionPoseSupplier;
        
        isSimulation = Robot.isSimulation();
        if (isSimulation) {
            visionSystemSim = new VisionSystemSim("main");
            visionSystemSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (CameraConfig cameraConfig : cameraConfigs) {
            String name = cameraConfig.name();
            PhotonCamera camera = new PhotonCamera(name);
            Matrix<N3, N1> specificStdDevs = baseStdDevs;
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                cameraConfig.apriltagLayout(),
                cameraConfig.strategy(),
                cameraConfig.robotToCamera()
            );
       
            cameras.put(name, camera);
            stdDevs.put(name, specificStdDevs);
            estimators.put(name, estimator);

            if (isSimulation) {
                PhotonCameraSim cameraSim = new PhotonCameraSim(camera, simCameraProperties);
                    cameraSim.enableDrawWireframe(false);
                    cameraSim.enableRawStream(true);
                    cameraSim.enableProcessedStream(true);
                visionSystemSim.addCamera(cameraSim, cameraConfig.robotToCamera());
            }

            System.out.println("Camera has been loaded: " + name);
        } 
    }

    @Override
    public void periodic() {
        switch(visionState) {
            case GLOBAL:
                for (Map.Entry<String, PhotonCamera> entry : cameras.entrySet()) {
                    currentCamera = entry.getValue();
                    currentEstimator = estimators.get(entry.getKey());

                    List<PhotonPipelineResult> pipelineResults = currentCamera.getAllUnreadResults();
                    if (!pipelineResults.isEmpty() && pipelineResults != null) {
                        for (PhotonPipelineResult currentResult : pipelineResults) {
                            if (currentResult.hasTargets()) {
                                amountOfTagsUsed = 0;
                                totalDistance = 0;
                                translationStdDevs = 0;
                                rotationStdDevs = 0;
                                visibleTagPoses = new ArrayList<>();

                                currentEstimatedRobotPose = currentEstimator.update(currentResult).get();
                                currentRobotPose = currentEstimatedRobotPose.estimatedPose;
                                    if (Math.abs(currentRobotPose.getZ()) > 0.25) continue;

                                amountOfTagsUsed = currentEstimatedRobotPose.targetsUsed.size();
                                for (PhotonTrackedTarget currentTrackedTarget : currentEstimatedRobotPose.targetsUsed) {
                                    trackedTagId = currentTrackedTarget.fiducialId;

                                    currentDistanceToTarget = currentTrackedTarget.getBestCameraToTarget().getTranslation().getNorm();
                                    if (amountOfTagsUsed == 1) trustedDistance = Units.feetToMeters(10);
                                        else trustedDistance = Units.feetToMeters(15);

                                    if (currentDistanceToTarget > trustedDistance) continue;
                                        else amountOfTrustedTags++;

                                    totalDistance += currentDistanceToTarget;
                                    APRIL_TAG_FIELD_LAYOUT.getTagPose(trackedTagId).ifPresent(pose -> visibleTagPoses.add(pose));
                                }

                                avgDistance = totalDistance / amountOfTagsUsed;
                                if (amountOfTrustedTags == 0) {
                                    continue;
                                } else if (amountOfTrustedTags == 1) {
                                    translationStdDevs = 0.7;
                                    rotationStdDevs = 0.7;
                                } else {
                                    translationStdDevs = 0.7;
                                    rotationStdDevs = 0.7;
                                }

                                visionPoseTimestamp = Utils.fpgaToCurrentTime(currentEstimatedRobotPose.timestampSeconds);
                                currentStdDevs = VecBuilder.fill(translationStdDevs, translationStdDevs, rotationStdDevs);
                                this.visionConsumer.accept(
                                    currentRobotPose.toPose2d(),
                                    visionPoseTimestamp,
                                    currentStdDevs
                                );

                                rawVisionPosePublisher.accept(currentRobotPose.toPose2d());
                                trackedTagsPublisher.accept(visibleTagPoses.toArray(new Pose3d[0]));
                                compensatedVisionPosePublisher.accept(pastVisionPoseSupplier.get());
                            }
                        }
                    } 
                    if (isSimulation) {
                        if (poseSupplier != null) visionSystemSim.update(poseSupplier.get());
                    }
                }
                break;
        }
    }   

    /**
     * Changes a specific camera's pose estimator
     * to use a new specified apriltag field layout
     * 
     * @param name name of camera (how we reference the specific estimator)
     * @param field the new list of apriltags + their positions on the field
     */
    public void changeFieldLayout(String name, AprilTagFields field) {
        estimators.get(name).setFieldTags(AprilTagFieldLayout.loadField(field));
    }

    /** 
     * Changes a specific camera's pose estimator
     * to use a new specified pose estimation strategy
     * 
     * @param name name of camera (how we reference the specific estimator)
     * @param strategy the new pose estimation strategy
     */
    public void changePoseStrategy(String name, PoseStrategy strategy) {
        estimators.get(name).setPrimaryStrategy(strategy);
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
        );
    }
}