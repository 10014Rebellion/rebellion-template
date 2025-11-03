// REBELLION 10014

package frc.robot.systems.vision;

import static frc.robot.systems.vision.VisionConstants.KUseSingleTagTransform;
import static frc.robot.systems.vision.VisionConstants.kAmbiguityThreshold;
import static frc.robot.systems.vision.VisionConstants.kMultiStdDevs;
import static frc.robot.systems.vision.VisionConstants.kSingleStdDevs;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.tuning.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Vision {
    private CameraIO[] cameras;
    private CameraIOInputsAutoLogged[] camerasData;

    private static final LoggedTunableNumber kSingleXYStdev = new LoggedTunableNumber("Vision/kSingleXYStdev",
            kSingleStdDevs.get(0));
    private static final LoggedTunableNumber kMultiXYStdev = new LoggedTunableNumber("Vision/kMultiXYStdev",
            kMultiStdDevs.get(0));

    public static final AprilTagFieldLayout k2025Field = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public Vision(CameraIO[] cameras) {
        Logger.recordOutput("Vision/UseSingleTagTransform", KUseSingleTagTransform);
        this.cameras = cameras;
        camerasData = new CameraIOInputsAutoLogged[cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            camerasData[i] = new CameraIOInputsAutoLogged();
        }
    }

    public void periodic(Pose2d lastRobotPose, Pose2d simOdomPose) {
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].updateInputs(camerasData[i], lastRobotPose, simOdomPose);
            Logger.processInputs("Vision/" + camerasData[i].camName, camerasData[i]);
        }
    }

    public VisionObservation[] getVisionObservations() {
        VisionObservation[] observations = new VisionObservation[cameras.length];
        for (int i = 0; i < camerasData.length; i++) {
            observations[i] = processCameraObservation(camerasData[i]);
        }
        return observations;
    }
    

    private VisionObservation processCameraObservation(CameraIOInputsAutoLogged camData) {
        if (!camData.hasTarget || !camData.hasBeenUpdated) {
            return makeInvalidObservation(camData);
        }
    
        int usableTags = 0;
        double totalDistance = 0.0;
    
        for (int i = 0; i < camData.latestTagTransforms.length; i++) {
            if (camData.latestTagTransforms[i] != null &&
                camData.latestTagAmbiguities[i] < kAmbiguityThreshold) {
                totalDistance += camData.latestTagTransforms[i].getTranslation().getNorm();
                usableTags++;
            }
        }
    
        if (usableTags == 0) {
            return makeUntrustedObservation(camData);
        }
    
        double avgDist = totalDistance / usableTags;
        double xyScalar = Math.pow(avgDist, 2) / usableTags;
    
        if (usableTags == 1) {
            return processSingleTagObservation(camData, avgDist, xyScalar);
        } else {
            return makeVisionObservation(camData.latestEstimatedRobotPose.toPose2d(),
                    kMultiXYStdev.get() * xyScalar, camData);
        }
    }

    private VisionObservation processSingleTagObservation(CameraIOInputsAutoLogged camData, double avgDist, double xyScalar) {
        if (avgDist > VisionConstants.kMaxTrustDistance) {
            return makeUntrustedObservation(camData);
        }
    
        Pose2d pose;
        if (KUseSingleTagTransform) {
            Optional<Pose3d> tagPoseOpt = k2025Field.getTagPose(camData.singleTagAprilTagID);
            if (tagPoseOpt.isEmpty()) {
                DriverStation.reportWarning("<<< COULD NOT FIND TAG ON FIELD! >>>", true);
                return makeUntrustedObservation(camData);
            }
            pose = tagPoseOpt.get().toPose2d()
                    .plus(new Transform2d(
                            camData.cameraToApriltag.getX(),
                            camData.cameraToApriltag.getY(),
                            camData.cameraToApriltag.getRotation().toRotation2d()))
                    .plus(toTransform2d(camData.cameraToRobot.inverse()));
        } else {
            pose = camData.latestEstimatedRobotPose.toPose2d();
        }
    
        return makeVisionObservation(pose, kSingleXYStdev.get() * xyScalar, camData);
    }
    
    

    private Transform2d toTransform2d(Transform3d transform) {
        return new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
    }

    public void logVisionObservation(VisionObservation observation, String state) {
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/State", state);
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/Timestamp", observation.timeStamp());
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/Pose", observation.pose());
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/hasObserved", observation.hasObserved());
        Logger.recordOutput("Vision/Observation/" + observation.camName + "/StdDevs", observation.stdDevs());
    }

    private VisionObservation makeVisionObservation(Pose2d pose, double xyStdev, CameraIOInputsAutoLogged camData) {
        return new VisionObservation(
                true,
                pose,
                VecBuilder.fill(xyStdev, xyStdev, Double.MAX_VALUE),
                camData.latestTimestamp,
                camData.camName);
    }
    
    private VisionObservation makeUntrustedObservation(CameraIOInputsAutoLogged camData) {
        return new VisionObservation(
                true,
                camData.latestEstimatedRobotPose.toPose2d(),
                VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
                camData.latestTimestamp,
                camData.camName);
    }
    
    private VisionObservation makeInvalidObservation(CameraIOInputsAutoLogged camData) {
        return new VisionObservation(
                false,
                new Pose2d(),
                VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE),
                camData.latestTimestamp,
                camData.camName);
    }
    

    public record VisionObservation(boolean hasObserved, Pose2d pose, Vector<N3> stdDevs, double timeStamp,
            String camName) {
    }
}
