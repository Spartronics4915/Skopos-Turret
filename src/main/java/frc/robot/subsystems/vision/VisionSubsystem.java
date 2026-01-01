package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.*;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {

    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private List<PhotonPipelineResult> results;
    private Optional<EstimatedRobotPose> currentResultPose;
    private Matrix<N3, N1> currentStdDevs;
    private Pose2d visionPose2d;

    StructPublisher<Pose2d> visionPosePublisher = NetworkTableInstance.getDefault().getStructTopic("VisionPose", Pose2d.struct).publish();

    public VisionSubsystem() {
        camera = new PhotonCamera("daniil");
        estimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), 
            PNP_DISTANCE_TRIG_SOLVE, 
            ROBOT_TO_CAMERA
        );
        System.out.println("Camera and Estimator loaded");
    }

    @Override
    public void periodic() {
        if (!camera.getAllUnreadResults().isEmpty()) {
            results = camera.getAllUnreadResults();
            for (PhotonPipelineResult currentResult : results) {
                currentResultPose = estimator.update(currentResult);
                System.out.println("Pose Acquired");
                updateEstimationStdDevs(currentResultPose, currentResult.getTargets());
                System.out.println("Standard Deviations Updated");
                currentResultPose.ifPresent(
                    estimate -> {
                        visionPose2d = estimate.estimatedPose.toPose2d();
                    }
                );
            }
            visionPosePublisher.accept(visionPose2d);
        }
    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) currentStdDevs = SINGLE_TAG_STD_DEVS;
            else {
                Matrix<N3, N1> estimateStdDevs = SINGLE_TAG_STD_DEVS;
                int numTags = 0;
                double avgDist = 0;
                for (PhotonTrackedTarget trackedTarget : targets) {
                    Optional<Pose3d> tagPose = estimator.getFieldTags().getTagPose(trackedTarget.getFiducialId());
                    if (tagPose.isEmpty()) continue;
                    numTags++;
                    avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                }
                if (numTags == 0) currentStdDevs = SINGLE_TAG_STD_DEVS;
                    else {
                        avgDist /= numTags;
                        if (numTags > 1) estimateStdDevs = MULTI_TAG_STD_DEVS;
                        if (numTags == 1 && avgDist > 4) estimateStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                            else estimateStdDevs = estimateStdDevs.times(1 + (avgDist * avgDist / 30));
                        currentStdDevs = estimateStdDevs;
                    }
            }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }

    public Pose3d getPose2d(EstimatedRobotPose estimatedRobotPose) {
        return estimatedRobotPose.estimatedPose;
    }
}
