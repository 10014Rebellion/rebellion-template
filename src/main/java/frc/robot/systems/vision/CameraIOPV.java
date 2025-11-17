// REBELLION 10014

package frc.robot.systems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.game.FieldConstants;
import frc.robot.systems.vision.VisionConstants.CameraSimConfigs;
import frc.robot.systems.vision.VisionConstants.Orientation;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraIOPV implements CameraIO {
    private String mCamName;
    private PhotonCamera mPhotonCam;
    private PhotonPoseEstimator mPoseEstimator;
    private Transform3d mCameraTransform;
    private final Orientation mOrientation;

    private VisionSystemSim mVisionSim;
    private PhotonCameraSim mCameraSim;

    public CameraIOPV(String pName, Transform3d pCameraTransform, Orientation pOrientation) {
        this.mCamName = pName;
        this.mPhotonCam = new PhotonCamera(mCamName);
        this.mCameraTransform = pCameraTransform;
        this.mOrientation = pOrientation;

        PhotonCamera.setVersionCheckEnabled(false); // Avoid version spam

        mPoseEstimator = new PhotonPoseEstimator(
                FieldConstants.kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, pCameraTransform);

        mPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

        if (Constants.currentMode == Mode.SIM) {
            setupSimulation();
        }
    }

    private void setupSimulation() {
        mVisionSim = new VisionSystemSim("main");
        mVisionSim.addAprilTags(FieldConstants.kFieldLayout);

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(
                (int) CameraSimConfigs.resWidth.value,
                (int) CameraSimConfigs.resHeight.value,
                new Rotation2d(CameraSimConfigs.fovDeg.value));
        cameraProps.setCalibError(CameraSimConfigs.avgErrorPx.value, CameraSimConfigs.errorStdDevPx.value);
        cameraProps.setFPS(CameraSimConfigs.fps.value);
        cameraProps.setAvgLatencyMs(CameraSimConfigs.avgLatencyMs.value);
        cameraProps.setLatencyStdDevMs(CameraSimConfigs.latencyStdDevMs.value);

        mCameraSim = new PhotonCameraSim(mPhotonCam, cameraProps);
        mVisionSim.addCamera(mCameraSim, mCameraTransform);
    }

    @Override
    public void updateInputs(CameraIOInputs pInputs, Pose2d pLastRobotPose, Pose2d pSimOdomPose) {
        pInputs.iCamName = mCamName;
        pInputs.iCameraToRobot = mCameraTransform;

        try {
            if (Constants.currentMode == Mode.SIM) {
                mVisionSim.update(pSimOdomPose);
            }

            List<PhotonPipelineResult> unreadResults = mPhotonCam.getAllUnreadResults();
            mPoseEstimator.setLastPose(pLastRobotPose);
            pInputs.iHasBeenUpdated = !unreadResults.isEmpty();

            if (!pInputs.iHasBeenUpdated) return;

            PhotonPipelineResult latestValidResult = null;
            Optional<EstimatedRobotPose> latestEstimatedRobotPose = Optional.empty();

            for (PhotonPipelineResult r : unreadResults) {
                Optional<EstimatedRobotPose> maybePose = mPoseEstimator.update(r);
                if (maybePose.isPresent()) {
                    latestValidResult = r;
                    latestEstimatedRobotPose = maybePose;
                }
            }

            pInputs.iIsConnected = mPhotonCam.isConnected();

            if (latestValidResult == null || !latestValidResult.hasTargets()) {
                DriverStation.reportWarning(
                        "No valid pose found in unread PhotonVision results for " + mCamName, false);
                pInputs.iHasTarget = false;
                return;
            }

            pInputs.iHasTarget = true;

            PhotonTrackedTarget target = latestValidResult.getBestTarget();
            pInputs.iCameraToApriltag = target.getBestCameraToTarget();
            pInputs.iRobotToApriltag = target.getBestCameraToTarget().plus(mCameraTransform);
            pInputs.iSingleTagAprilTagID = target.getFiducialId();
            pInputs.iPoseAmbiguity = target.getPoseAmbiguity();
            pInputs.iYaw = target.getYaw();
            pInputs.iPitch = target.getPitch();
            pInputs.iArea = target.getArea();
            pInputs.iLatencySeconds = latestValidResult.metadata.getLatencyMillis() / 1000.0;
            pInputs.iLatestTimestamp = latestValidResult.getTimestampSeconds();

            latestEstimatedRobotPose.ifPresent(est -> {
                if (mOrientation == Orientation.BACK)
                    pInputs.iLatestEstimatedRobotPose = est.estimatedPose.transformBy(
                            new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, Math.PI)));
                else pInputs.iLatestEstimatedRobotPose = est.estimatedPose;

                int count = est.targetsUsed.size();
                Transform3d[] tagTransforms = new Transform3d[count];
                double[] ambiguities = new double[count];

                for (int i = 0; i < count; i++) {
                    tagTransforms[i] = est.targetsUsed.get(i).getBestCameraToTarget();
                    ambiguities[i] = est.targetsUsed.get(i).getPoseAmbiguity();
                }

                pInputs.iNumberOfTargets = count;
                pInputs.iLatestTagTransforms = tagTransforms;
                pInputs.iLatestTagAmbiguities = ambiguities;
            });

        } catch (Exception e) {
            e.printStackTrace();
            resetInputs(pInputs);
        }
    }

    private void resetInputs(CameraIOInputs pInputs) {
        pInputs.iIsConnected = false;
        pInputs.iHasTarget = false;
        pInputs.iHasBeenUpdated = false;
        pInputs.iYaw = 0.0;
        pInputs.iPitch = 0.0;
        pInputs.iArea = 0.0;
        pInputs.iLatencySeconds = 0.0;
        pInputs.iPoseAmbiguity = 0.0;
        pInputs.iSingleTagAprilTagID = 0;
        pInputs.iNumberOfTargets = 0;
        pInputs.iLatestTimestamp = 0.0;
        pInputs.iCameraToApriltag = new Transform3d();
        pInputs.iRobotToApriltag = new Transform3d();
        pInputs.iLatestEstimatedRobotPose = new Pose3d();
        pInputs.iLatestTagTransforms = new Transform3d[0];
        pInputs.iLatestTagAmbiguities = new double[0];
    }
}
