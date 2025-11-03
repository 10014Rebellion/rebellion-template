package frc.robot.systems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.systems.vision.VisionConstants.CameraSimConfigs;
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

    private VisionSystemSim mVisionSim;
    private PhotonCameraSim mCameraSim;

    public CameraIOPV(String pName, Transform3d pCameraTransform) {
        this.mCamName = pName;
        this.mPhotonCam = new PhotonCamera(mCamName);
        this.mCameraTransform = pCameraTransform;
        
        PhotonCamera.setVersionCheckEnabled(false); // Avoid version spam

        mPoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            pCameraTransform
        );

        mPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);

        if (Constants.currentMode == Mode.SIM) {
            setupSimulation();
        }
    }
    
    private void setupSimulation() {
        mVisionSim = new VisionSystemSim("main");
        mVisionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));

        SimCameraProperties cameraProps = new SimCameraProperties();
        cameraProps.setCalibration(
            (int) CameraSimConfigs.resWidth.value, 
            (int) CameraSimConfigs.resHeight.value, 
            new Rotation2d(CameraSimConfigs.fovDeg.value)
        );
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

            // TODO: Don't only get the last result
            PhotonPipelineResult result = unreadResults.get(unreadResults.size() - 1);
            Optional<EstimatedRobotPose> latestEstimatedRobotPose = mPoseEstimator.update(result);

            pInputs.iIsConnected = mPhotonCam.isConnected();
            pInputs.iHasTarget = result.hasTargets();

            if (!pInputs.iHasTarget) return;

            PhotonTrackedTarget target = result.getBestTarget();
            pInputs.iCameraToApriltag = target.getBestCameraToTarget();
            pInputs.iRobotToApriltag = target.getBestCameraToTarget().plus(mCameraTransform);
            pInputs.iSingleTagAprilTagID = target.getFiducialId();
            pInputs.iPoseAmbiguity = target.getPoseAmbiguity();
            pInputs.iYaw = target.getYaw();
            pInputs.iPitch = target.getPitch();
            pInputs.iArea = target.getArea();
            pInputs.iLatencySeconds = result.metadata.getLatencyMillis() / 1000.0;

            latestEstimatedRobotPose.ifPresent(est -> {
                pInputs.iLatestEstimatedRobotPose = est.estimatedPose;

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
                pInputs.iLatestTimestamp = result.getTimestampSeconds();
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
