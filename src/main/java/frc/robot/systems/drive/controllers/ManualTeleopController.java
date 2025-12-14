package frc.robot.systems.drive.controllers;

import static frc.robot.bindings.BindingsConstants.kDefaultProfile;
import static frc.robot.systems.drive.DriveConstants.kMaxLinearSpeedMPS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.systems.drive.DriveConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/* Controls the pose of the robot using 3 PID controllers and Feedforwards */
public class ManualTeleopController {
    public static TuneableDriverProfile mDriverProfile = new TuneableDriverProfile(kDefaultProfile);

    private DoubleSupplier mXSupplier;
    private DoubleSupplier mYSupplier;
    private DoubleSupplier mOmegaSupplier;

    private Supplier<Rotation2d> mPOVSupplier;

    public ManualTeleopController() {}

    public void acceptJoystickInputs(
            DoubleSupplier pXSupplier,
            DoubleSupplier pYSupplier,
            DoubleSupplier pOmegaSupplier,
            Supplier<Rotation2d> pPOVSupplier) {
        this.mXSupplier = pXSupplier;
        this.mYSupplier = pYSupplier;
        this.mOmegaSupplier = pOmegaSupplier;
        this.mPOVSupplier = pPOVSupplier;
    }

    public ChassisSpeeds computeChassiSpeeds(
            Rotation2d pRobotAngle, ChassisSpeeds pCurrentRobotRelativeSpeeds, boolean pIsJoystickSniper, boolean pIsJoystickFieldOriented) {
        double xAdjustedJoystickInput = MathUtil.applyDeadband(
                mXSupplier.getAsDouble(), mDriverProfile.linearDeadBand().get());
        double yAdjustedJoystickInput = MathUtil.applyDeadband(
                mYSupplier.getAsDouble(), mDriverProfile.linearDeadBand().get());
        double omegaAdjustedJoystickInput = MathUtil.applyDeadband(
                mOmegaSupplier.getAsDouble(), mDriverProfile.rotationDeadband().get());

        int linearExp = (int) Math.round(mDriverProfile.linearInputsExponent().get());
        int rotationExp = (int) Math.round(mDriverProfile.rotationInputsExponent().get());

        double xJoystickScalar = getSniperScalar(pIsJoystickSniper) * mDriverProfile.linearScalar().get();
        double yJoystickScalar = getSniperScalar(pIsJoystickSniper) * mDriverProfile.linearScalar().get();
        double omegaJoystickScalar = getSniperScalar(pIsJoystickSniper) * mDriverProfile.linearScalar().get();

        double xScaledJoystickInput = zerotoOneClamp(xJoystickScalar) * Math.pow(xAdjustedJoystickInput, linearExp);
        double yScaledJoystickInput = zerotoOneClamp(yJoystickScalar) * Math.pow(yAdjustedJoystickInput, linearExp);
        double omegaJoystickInput = zerotoOneClamp(omegaJoystickScalar) * Math.pow(omegaAdjustedJoystickInput, rotationExp);

        if (linearExp % 2 == 0) {
            xScaledJoystickInput *= Math.signum(xAdjustedJoystickInput);
            yScaledJoystickInput *= Math.signum(yAdjustedJoystickInput);
        }

        if (rotationExp % 2 == 0) {
            omegaJoystickInput *= Math.signum(omegaAdjustedJoystickInput);
        }

        if (DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red))
            pRobotAngle = pRobotAngle.plus(Rotation2d.k180deg);

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            DriveConstants.kMaxLinearSpeedMPS * xScaledJoystickInput,
            DriveConstants.kMaxLinearSpeedMPS * yScaledJoystickInput,
            DriveConstants.kMaxRotationSpeedRadiansPS * omegaJoystickInput
        );

        if (pIsJoystickFieldOriented) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, pRobotAngle);
        }

        return desiredSpeeds;
    }

    public double getSniperScalar(boolean pIsSniper) {
        return pIsSniper ? mDriverProfile.sniperControl().get() : 1.0;
    }

    public ChassisSpeeds computeSniperPOVChassisSpeeds(Rotation2d pRobotAngle, boolean pIsPOVFieldOriented) {
        double omegaAdjustedJoystickInput = MathUtil.applyDeadband(mOmegaSupplier.getAsDouble(), mDriverProfile.rotationDeadband().get());

        int rotationExp = (int) Math.round(mDriverProfile.rotationInputsExponent().get());

        double omegaJoystickScalar = getSniperScalar(true) * mDriverProfile.linearScalar().get();

        double omegaJoystickInput =
                zerotoOneClamp(omegaJoystickScalar) * Math.pow(omegaAdjustedJoystickInput, rotationExp);

        if (rotationExp % 2 == 0) {
            omegaJoystickInput *= Math.signum(omegaAdjustedJoystickInput);
        }

        /* Does polar to rectangular where POV degree is theta, kSniperControl * maxSpeed is r */ 
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
                -mDriverProfile.sniperControl().get()
                        * kMaxLinearSpeedMPS
                        * Math.cos(mPOVSupplier.get().getRadians()),
                mDriverProfile.sniperControl().get()
                        * kMaxLinearSpeedMPS
                        * Math.sin(mPOVSupplier.get().getRadians()),
                DriveConstants.kMaxRotationSpeedRadiansPS * omegaJoystickInput);

        if (pIsPOVFieldOriented) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, pRobotAngle);
        }

        return desiredSpeeds;
    }

    private double zerotoOneClamp(double pVal) {
        return MathUtil.clamp(pVal, 0.0, 1.0);
    }

    public void updateTuneablesWithProfiles(DriverProfiles pProfiles) {
        mDriverProfile = new TuneableDriverProfile(pProfiles);
    }

    public TuneableDriverProfile getmDriverProfile() {
        return mDriverProfile;
    }

    public static record DriverProfiles(
            double linearScalar,
            double linearExponent,
            double linearDeadband,
            double rotationalScalar,
            double rotationalExponent,
            double rotationDeadband,
            double sniperScalar,
            boolean swapSides,
            String key) {}
}
