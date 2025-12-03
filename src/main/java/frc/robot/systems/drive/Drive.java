// REBELLION 10014

package frc.robot.systems.drive;

import static frc.robot.systems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.GeomUtil;
import frc.lib.pathplanner.SwerveSetpoint;
import frc.lib.pathplanner.SwerveSetpointGenerator;
import frc.lib.swerve.LocalADStarAK;
import frc.lib.swerve.SwerveUtils;
import frc.lib.tuning.LoggedTunableNumber;
import frc.lib.tuning.SysIDCharacterization;
import frc.robot.game.FieldConstants;
import frc.robot.game.GameDriveManager;
import frc.robot.game.GameDriveManager.GameDriveStates;
import frc.robot.game.GameGoalPoseChooser;
import frc.robot.systems.drive.controllers.HeadingController;
import frc.robot.systems.drive.controllers.HolonomicController;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;
import frc.robot.systems.drive.controllers.ManualTeleopController;
import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;
import frc.robot.systems.drive.gyro.GyroIO;
import frc.robot.systems.drive.gyro.GyroInputsAutoLogged;
import frc.robot.systems.drive.modules.Module;
import frc.robot.systems.vision.Vision;
import frc.robot.systems.vision.Vision.VisionObservation;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/*
 * This code is a swerve drivebase
 * Main logic is handled in periodic() function
 */
public class Drive extends SubsystemBase {
    public static enum DriveState {
        // TELEOP AND AUTON CONTROLS
        TELEOP,
        TELEOP_SNIPER,
        POV_SNIPER,
        HEADING_ALIGN,
        AUTO_ALIGN,
        LINE_ALIGN, // TODO: Implement this.
        AUTON,
        STOP,

        // TUNING
        DRIFT_TEST,
        LINEAR_TEST,
        SYSID_CHARACTERIZATION,
        WHEEL_CHARACTERIZATION
    }

    /* HARDWARE LAYERS */
    private Module[] mModules;
    private GyroIO mGyro;
    private GyroInputsAutoLogged mGyroInputs = new GyroInputsAutoLogged();
    private Vision mVision;

    /* LOCALIZATION(tracks position and orientation of robot) */
    private Rotation2d mRobotRotation;
    private SwerveDriveOdometry mOdometry;
    private SwerveDrivePoseEstimator mPoseEstimator;
    private Field2d mField = new Field2d();

    /* PATHPLANNER AND SETPOINT GENERATOR(USED IN TELEOP) */
    public static RobotConfig mRobotConfig;
    private final SwerveSetpointGenerator mSetpointGenerator;
    private SwerveSetpoint mPreviousSetpoint =
            new SwerveSetpoint(new ChassisSpeeds(), SwerveUtils.zeroStates(), DriveFeedforwards.zeros(4));

    /* STATE OF DRIVEBASE */
    @AutoLogOutput(key = "Drive/State")
    private DriveState mDriveState = DriveState.TELEOP;

    private boolean mUseGenerator = true;

    /* SETPOINTS */
    private ChassisSpeeds mDesiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds mPPDesiredSpeeds = new ChassisSpeeds();
    private DriveFeedforwards mPathPlanningFF = DriveFeedforwards.zeros(4);
    private PathConstraints mDriveConstraints = DriveConstants.kAutoDriveConstraints;

    private SwerveModuleState[] mPrevStates = SwerveUtils.zeroStates();

    /* CONTROLLERS(are used to set chassis speeds) */
    private ManualTeleopController mTeleopController = new ManualTeleopController();

    private HeadingController mHeadingController = new HeadingController();

    @AutoLogOutput(key = "Drive/HeadingController/GoalRotation")
    private Supplier<Rotation2d> mGoalRotationSup = () -> new Rotation2d();

    private HolonomicController mAutoAlignController = new HolonomicController();

    @AutoLogOutput(key = "Drive/HeadingController/GoalPose")
    private Supplier<Pose2d> mGoalPoseSup = () -> new Pose2d();

    private GameDriveManager mGameDriveManager = new GameDriveManager(this);

    /* TUNABLE NUMBERS FOR DRIVEBASE CONSTANTS AND TESTS */
    private static final LoggedTunableNumber tDriftRate =
            new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);
    private static final LoggedTunableNumber tRotationDriftTestSpeedDeg =
            new LoggedTunableNumber("Drive/DriftRotationTestDeg", 360);
    private static final LoggedTunableNumber tLinearTestSpeedMPS = new LoggedTunableNumber("Drive/LinearTestMPS", 4.5);

    private final Debouncer mAutoAlignTimeout = new Debouncer(0.1, DebounceType.kRising);

    public Drive(Module[] modules, GyroIO gyro, Vision vision) {
        this.mModules = modules;
        this.mGyro = gyro;
        this.mVision = vision;

        mRobotRotation = mGyroInputs.yawPosition;

        mOdometry = new SwerveDriveOdometry(kKinematics, getmRobotRotation(), getModulePositions());
        mPoseEstimator =
                new SwerveDrivePoseEstimator(kKinematics, getmRobotRotation(), getModulePositions(), new Pose2d());
        try {
            /* Incase if pathplanner doesn't currently load */
            mRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println(
                    "<< PATH PLANNER SETTINGS DID NOT CORRECTLY LOAD. PLEASE MAKE SURE PATH PLANNER IS INSTALLED AND UP TO DATE >>>");
        }

        mSetpointGenerator = new SwerveSetpointGenerator(
                mRobotConfig, // The robot configuration. This is the same config for pathplanner as well
                kMaxAzimuthAngularRadiansPS // The max rotation velocity of a swerve module in radians per second.
                );

        // 4.29
        /* Sets up pathplanner to load paths. No need for choreo set-up as its handled internally by Pathplanner */
        /* Refer to getTrajectory() in AutonCommands */
        AutoBuilder.configure(
                this::getPoseEstimate,
                this::setPose,
                this::getRobotChassisSpeeds,
                (speeds, ff) -> {
                    mDriveState = DriveState.AUTON;
                    mPPDesiredSpeeds = new ChassisSpeeds(
                            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
                    mPathPlanningFF = ff;
                },
                new PPHolonomicDriveController(kPPTranslationPID, kPPRotationPID),
                mRobotConfig,
                () -> DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
                this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) ->
                Logger.recordOutput("Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("Drive/Odometry/TrajectorySetpoint", targetPose));

        SmartDashboard.putData(mField);

        mHeadingController.setHeadingGoal(mGoalRotationSup);
    }

    @Override
    public void periodic() {
        for (Module module : mModules) module.periodic();

        updateSensorsAndOdometry();
        updateDriveControllers();
        computeDesiredSpeeds();

        if (mDesiredSpeeds != null) runSwerve(mDesiredSpeeds);
    }

    private void updateSensorsAndOdometry() {
        /* GYRO */
        mGyro.updateInputs(mGyroInputs);
        Logger.processInputs("Drive/Gyro", mGyroInputs);
        if (mGyroInputs.connected) mRobotRotation = mGyroInputs.yawPosition;
        else
            mRobotRotation = Rotation2d.fromRadians(
                    (mPoseEstimator.getEstimatedPosition().getRotation().getRadians()
                                    /* D=vt. Uses modules and IK to estimate turn */
                                    + getRobotChassisSpeeds().omegaRadiansPerSecond * 0.02)
                            /* Scopes result between 0 and 360 */
                            % 360.0);

        /* VISION */
        mVision.periodic(mPoseEstimator.getEstimatedPosition(), mOdometry.getPoseMeters());
        VisionObservation[] observations = mVision.getVisionObservations();
        for (VisionObservation observation : observations) {
            if (observation.hasObserved())
                mPoseEstimator.addVisionMeasurement(observation.pose(), observation.timeStamp(), observation.stdDevs());

            Logger.recordOutput(
                    observation.camName() + "/stdDevX", observation.stdDevs().get(0));
            Logger.recordOutput(
                    observation.camName() + "/stdDevY", observation.stdDevs().get(1));
            Logger.recordOutput(
                    observation.camName() + "/stdDevTheta",
                    observation.stdDevs().get(2));
        }

        mPoseEstimator.update(mRobotRotation, getModulePositions());
        mOdometry.update(mRobotRotation, getModulePositions());

        mField.setRobotPose(getPoseEstimate());
    }

    private void updateDriveControllers() {
        mHeadingController.updateHeadingController();
        mAutoAlignController.updateAlignmentControllers();
        GameGoalPoseChooser.updateSideStuff();
    }

    private void computeDesiredSpeeds() {
        ChassisSpeeds teleopSpeeds =
                mTeleopController.computeChassiSpeeds(getPoseEstimate().getRotation(), getRobotChassisSpeeds(), false);
        switch (mDriveState) {
            case TELEOP:
                mDesiredSpeeds = teleopSpeeds;
                break;
            case TELEOP_SNIPER:
                mDesiredSpeeds = mTeleopController.computeChassiSpeeds(
                        getPoseEstimate().getRotation(), getRobotChassisSpeeds(), true);
                break;
            case POV_SNIPER:
                mDesiredSpeeds = mTeleopController.computeSniperPOVChassisSpeeds(
                        getPoseEstimate().getRotation(), false);
                break;
            case HEADING_ALIGN:
                mDesiredSpeeds = new ChassisSpeeds(
                        teleopSpeeds.vxMetersPerSecond,
                        teleopSpeeds.vyMetersPerSecond,
                        mHeadingController.getSnapOutput(getPoseEstimate().getRotation()));
                break;
            case AUTO_ALIGN:
                mDesiredSpeeds = mAutoAlignController.calculate(mGoalPoseSup.get(), getPoseEstimate());
                break;
            case AUTON:
                mDesiredSpeeds = mPPDesiredSpeeds;
                break;
            case DRIFT_TEST:
                mDesiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(
                                tLinearTestSpeedMPS.get(), 0.0, Math.toRadians(tRotationDriftTestSpeedDeg.get())),
                        mRobotRotation);
                break;
            case LINEAR_TEST:
                mDesiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        new ChassisSpeeds(tLinearTestSpeedMPS.get(), 0.0, 0.0), mRobotRotation);
                break;
                /* Set by characterization commands in the CHARACTERIZATION header. Wheel characterization is currently unimplemented */
            case SYSID_CHARACTERIZATION:
            case WHEEL_CHARACTERIZATION:
                /* If null, then PID isn't set, so characterization can set motors w/o interruption */
                mDesiredSpeeds = null;
            case STOP:
                for (int i = 0; i < mModules.length; i++) {
                    mModules[i].setDesiredState(new SwerveModuleState(0.0, mModules[i].getCurrentState().angle));
                }
                break;
            default:
                /* Defaults to Teleop control if no other cases are run*/
        }
    }

    ///////////////////////// GAME STATES \\\\\\\\\\\\\\\\\\\\\\\\
    public Command getGameDriveCommand(GameDriveStates pGameDriveStates) {
        return mGameDriveManager.getSetGameDriveStateCmd(pGameDriveStates);
    }

    ///////////////////////// STATE SETTING \\\\\\\\\\\\\\\\\\\\\\\\
    public Command setToTeleop() {
        return setDriveStateCommandContinued(DriveState.TELEOP);
    }

    public Command setToTeleopSniper() {
        return setDriveStateCommandContinued(DriveState.TELEOP_SNIPER);
    }

    public Command setToPOVSniper() {
        return setDriveStateCommandContinued(DriveState.POV_SNIPER);
    }

    public Command setToStop() {
        return setDriveStateCommandContinued(DriveState.STOP);
    }

    public Command setToDriftTest() {
        return setDriveStateCommandContinued(DriveState.DRIFT_TEST);
    }

    public Command setToLinearTest() {
        return setDriveStateCommandContinued(DriveState.LINEAR_TEST);
    }

    public Command setToSysIDCharacterization() {
        return setDriveStateCommandContinued(DriveState.SYSID_CHARACTERIZATION);
    }

    public Command setToWheelCharacterization() {
        return setDriveStateCommandContinued(DriveState.WHEEL_CHARACTERIZATION);
    }

    public Command customFollowPathComamnd(PathPlannerPath path) {
        return customFollowPathComamnd(path, new PPHolonomicDriveController(kPPTranslationPID, kPPRotationPID));
    }

    public Command customFollowPathComamnd(PathPlannerPath path, PPHolonomicDriveController drivePID) {
        return new FollowPathCommand(
                path,
                this::getPoseEstimate,
                this::getRobotChassisSpeeds,
                (speeds, ff) -> {
                    setDriveState(DriveState.AUTON);
                    mPPDesiredSpeeds = speeds;
                    mPathPlanningFF = ff;
                },
                drivePID,
                mRobotConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red),
                this);
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of this command
     * @param Goal strategy, based on where you're aligning
     * @param Constraint type, linear or on an axis
     */
    public Command setToGenericAutoAlign(Supplier<Pose2d> pGoalPoseSup, ConstraintType pConstraintType) {
        return new InstantCommand(() -> {
                    mGoalPoseSup = pGoalPoseSup;
                    mAutoAlignController.setConstraintType(pConstraintType, getPoseEstimate(), mGoalPoseSup.get());
                    mAutoAlignController.reset(
                            getPoseEstimate(),
                            ChassisSpeeds.fromRobotRelativeSpeeds(
                                    getRobotChassisSpeeds(), getPoseEstimate().getRotation()),
                            mGoalPoseSup.get());
                })
                .andThen(setDriveStateCommandContinued(DriveState.AUTO_ALIGN));
    }

    /*
     * Reference GameDriveManager to use game-specific implementation of this command
     * @param The desired rotation
     */
    public Command setToGenericHeadingAlign(Supplier<Rotation2d> pGoalRotation) {
        return new InstantCommand(() -> {
                    mGoalRotationSup = pGoalRotation;
                    mHeadingController.setHeadingGoal(mGoalRotationSup);
                    mHeadingController.reset(getPoseEstimate().getRotation(), mGyroInputs.yawVelocityPS);
                })
                .andThen(setDriveStateCommandContinued(DriveState.HEADING_ALIGN));
    }

    ////// BASE STATES \\\\\\
    private Command setDriveStateCommand(DriveState state) {
        return Commands.runOnce(() -> setDriveState(state), this);
    }

    /* Set's state initially, and doesn't end till interruped by another drive command */
    private Command setDriveStateCommandContinued(DriveState state) {
        return new FunctionalCommand(() -> setDriveState(state), () -> {}, (interrupted) -> {}, () -> false, this);
    }

    /* Sets the drive state used in periodic(), and handles init condtions like resetting PID controllers */
    private void setDriveState(DriveState state) {
        mDriveState = state;
    }

    ////////////// CHASSIS SPEED TO MODULES \\\\\\\\\\\\\\\\
    /* Sets the desired swerve module states to the robot */
    public void runSwerve(ChassisSpeeds speeds) {
        mDesiredSpeeds = SwerveUtils.discretize(speeds, tDriftRate.get());

        /* Logs all the possible drive states, great for debugging */
        SwerveUtils.logPossibleDriveStates(
                kDoExtraLogging, mDesiredSpeeds, getModuleStates(), mPreviousSetpoint, mRobotRotation);

        SwerveModuleState[] unOptimizedSetpointStates = new SwerveModuleState[4];
        SwerveModuleState[] setpointStates = kKinematics.toSwerveModuleStates(mDesiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeedMPS);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        if (DriverStation.isAutonomous()) {
            mPreviousSetpoint =
                    mSetpointGenerator.generateSetpoint(mPreviousSetpoint, mDesiredSpeeds, kAutoDriveConstraints, 0.02);
        } else {
            mPreviousSetpoint =
                    mSetpointGenerator.generateSetpoint(mPreviousSetpoint, mDesiredSpeeds, mDriveConstraints, 0.02);
        }

        /* Only for logging purposes */
        SwerveModuleState[] moduleTorques = SwerveUtils.zeroStates();

        // Logger.recordOutput("Drive/Odometry/generatedFieldSpeeds",
        // ChassisSpeeds.fromRobotRelativeSpeeds(previousSetpoint.robotRelativeSpeeds(), robotRotation));

        for (int i = 0; i < 4; i++) {
            if (mUseGenerator) {
                /* Logs the drive feedforward stuff */
                SwerveUtils.logDriveFeedforward(mPreviousSetpoint.feedforwards(), i);

                setpointStates[i] = new SwerveModuleState(
                        mPreviousSetpoint.moduleStates()[i].speedMetersPerSecond,
                        /* setpointAngle = currentAngle if the speed is less than 0.01 */
                        SwerveUtils.removeAzimuthJitter(
                                mPreviousSetpoint.moduleStates()[i], mModules[i].getCurrentState()));

                unOptimizedSetpointStates[i] = SwerveUtils.copyState(setpointStates[i]);

                setpointStates[i].optimize(mModules[i].getCurrentState().angle);

                /* Feedforward cases based on driveState */
                /* 0 unless in auto or auto-align */
                double driveAmps = calculateDriveFeedforward(
                        mPreviousSetpoint,
                        mModules[i].getCurrentState(),
                        unOptimizedSetpointStates[i],
                        setpointStates[i],
                        i);

                /*
                 * Multiplies by cos(angleError) to stop the drive from going in the wrong direction
                 * when azimuth angle changes
                 */
                setpointStates[i].cosineScale(mModules[i].getCurrentState().angle);

                double directionOfVelChange =
                        Math.signum(setpointStates[i].speedMetersPerSecond - mPrevStates[i].speedMetersPerSecond);
                Logger.recordOutput("Drive/Module/Feedforward/" + i + "/dir", directionOfVelChange);
                if (mDriveState.equals(DriveState.AUTON)) {
                    driveAmps = Math.abs(driveAmps) * Math.signum(directionOfVelChange);
                }

                optimizedSetpointStates[i] = mModules[i].setDesiredStateWithAmpFF(setpointStates[i], driveAmps);

                moduleTorques[i] =
                        new SwerveModuleState((driveAmps * kMaxLinearSpeedMPS / 80), optimizedSetpointStates[i].angle);
            } else {
                setpointStates[i] = new SwerveModuleState(
                        setpointStates[i].speedMetersPerSecond,
                        SwerveUtils.removeAzimuthJitter(setpointStates[i], mModules[i].getCurrentState()));

                setpointStates[i].optimize(mModules[i].getCurrentState().angle);
                setpointStates[i].cosineScale(mModules[i].getCurrentState().angle);
                optimizedSetpointStates[i] = mModules[i].setDesiredState(setpointStates[i]);
            }
        }

        mPrevStates = optimizedSetpointStates;

        Logger.recordOutput("Drive/Swerve/Setpoints", unOptimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput(
                "Drive/Swerve/SetpointsChassisSpeeds", kKinematics.toChassisSpeeds(optimizedSetpointStates));
        Logger.recordOutput(
                "Drive/Odometry/FieldSetpointChassisSpeed",
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        kKinematics.toChassisSpeeds(optimizedSetpointStates), mRobotRotation));
        Logger.recordOutput("Drive/Swerve/ModuleTorqueFF", moduleTorques);
    }

    /* Calculates DriveFeedforward based off state */
    public double calculateDriveFeedforward(
            SwerveSetpoint setpoint,
            SwerveModuleState currentState,
            SwerveModuleState unoptimizedState,
            SwerveModuleState optimizedState,
            int i) {
        switch (mDriveState) {
            case AUTON:
                /* No need to optimize for Choreo, as it handles it under the hood */
                return SwerveUtils.convertChoreoNewtonsToAmps(currentState, mPathPlanningFF, i);
                // TODO: Fix this.
            case AUTO_ALIGN:
                return SwerveUtils.optimizeTorque(
                        unoptimizedState,
                        optimizedState,
                        setpoint.feedforwards().torqueCurrentsAmps()[i],
                        i);
            default:
                return 0.0;
        }
    }

    ////////////// LOCALIZATION(MAINLY RESETING LOGIC) \\\\\\\\\\\\\\\\
    public void resetGyro() {
        /* Robot is usually facing the other way(relative to field) when doing cycles on red side, so gyro is reset to 180 */
        mRobotRotation = DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get().equals(Alliance.Red)
                ? Rotation2d.fromDegrees(180.0)
                : Rotation2d.fromDegrees(0.0);
        mGyro.resetGyro(mRobotRotation);
        setPose(new Pose2d(new Translation2d(), mRobotRotation));
    }

    public void setPose(Pose2d pose) {
        setPoses(pose, pose);
    }

    public void setPoses(Pose2d estimatorPose, Pose2d odometryPose) {
        mRobotRotation = estimatorPose.getRotation();
        mGyro.resetGyro(mRobotRotation);
        // Safe to pass in odometry poses because of the syncing
        // between gyro and pose estimator in reset gyro function
        mPoseEstimator.resetPosition(getmRobotRotation(), getModulePositions(), estimatorPose);
        mOdometry.resetPosition(getmRobotRotation(), getModulePositions(), odometryPose);
    }

    public void resetModulesEncoders() {
        for (int i = 0; i < 4; i++) mModules[i].resetAzimuthEncoder();
    }

    public Command setDriveProfile(DriverProfiles profile) {
        return new InstantCommand(() -> {
            mTeleopController.updateTuneablesWithProfiles(profile);

            GameGoalPoseChooser.setSwapSides(profile.swapSides());
        });
    }

    //////////////////////// CHARACTERIZATION \\\\\\\\\\\\\\\\\\\\\\\\\\
    /* LINEAR CHARACTERIZATION: The x-y movement of the drivetrain(basically drive motor feedforward) */
    public Command characterizeLinearMotion() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION)
                .andThen(SysIDCharacterization.runDriveSysIDTests(
                        (voltage) -> {
                            runLinearCharcterization(voltage);
                        },
                        this));
    }

    /* Runs the robot forward at a voltage */
    public void runLinearCharcterization(double volts) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        for (int i = 0; i < 4; i++) mModules[i].runCharacterization(volts);
    }

    public void setForwardAmperagesForAllModules(double amps) {
        for (int i = 0; i < 4; i++) {
            mModules[i].setDesiredVelocity(null);
            mModules[i].setDriveAmperage(amps);
            mModules[i].setDesiredRotation(Rotation2d.fromDegrees(0.0));
        }
    }

    /*
     * ANGULAR CHARACTERIZATION: The angular movement of the drivetrain(basically used to get drivebase MOI)
     * https://choreo.autos/usage/estimating-moi/
     */
    public Command characterizeAngularMotion() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION)
                .andThen(SysIDCharacterization.runDriveSysIDTests(
                        (voltage) -> runAngularCharacterization(voltage), this));
    }

    /* Runs the rotate's robot at a voltage */
    public void runAngularCharacterization(double volts) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        mModules[0].runCharacterization(volts, Rotation2d.fromDegrees(-45.0));
        mModules[1].runCharacterization(-volts, Rotation2d.fromDegrees(45.0));
        mModules[2].runCharacterization(volts, Rotation2d.fromDegrees(45.0));
        mModules[3].runCharacterization(-volts, Rotation2d.fromDegrees(-45.0));
    }

    public void runMOICharacterization(double amps) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        mModules[0].setDriveAmperage(amps);
        mModules[1].setDriveAmperage(-amps);
        mModules[2].setDriveAmperage(amps);
        mModules[3].setDriveAmperage(-amps);

        mModules[0].setDesiredRotation(Rotation2d.fromDegrees(-45.0));
        mModules[1].setDesiredRotation(Rotation2d.fromDegrees(45.0));
        mModules[2].setDesiredRotation(Rotation2d.fromDegrees(45.0));
        mModules[3].setDesiredRotation(Rotation2d.fromDegrees(-45.0));

        Logger.recordOutput(
                "Drive/MOI/RadiansVelocity",
                mModules[0].getInputs().driveVelocityMPS / DriveConstants.kDrivebaseRadiusMeters);
        Logger.recordOutput(
                "Drive/MOI/DriveTorqueNM",
                (SwerveUtils.getTorqueOfKrakenDriveMotor(mModules[0].getInputs().driveTorqueCurrentAmps)
                                * kDriveMotorGearing
                                / kWheelRadiusMeters)
                        * kDrivebaseRadiusMeters);
    }

    ///////////////////////// GETTERS \\\\\\\\\\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = mModules[i].getCurrentState();
        return states;
    }

    @AutoLogOutput(key = "Drive/Swerve/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = mModules[i].getCurrentPosition();
        return positions;
    }

    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getPoseEstimate() {
        return (RobotBase.isReal()) ? mPoseEstimator.getEstimatedPosition() : getOdometryPose();
    }

    @AutoLogOutput(key = "Drive/Odometry/OdometryPose")
    public Pose2d getOdometryPose() {
        return mOdometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getmRobotRotation() {
        return mRobotRotation;
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotChassisSpeeds")
    public ChassisSpeeds getRobotChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return mDesiredSpeeds;
    }

    @AutoLogOutput(key = "Drive/Tolerance/HeadingController")
    public boolean inHeadingTolerance() {
        /* Accounts for angle wrapping issues with rotation 2D error */
        return GeomUtil.getSmallestChangeInRotation(mRobotRotation, mGoalRotationSup.get())
                        .getDegrees()
                < HeadingController.toleranceDegrees.get();
    }

    @AutoLogOutput(key = "Drive/Odometry/DistanceFromReef")
    public double distanceFromReefCenter() {
        return getPoseEstimate()
                .getTranslation()
                .getDistance(AllianceFlipUtil.apply(FieldConstants.kReefCenter).getTranslation());
    }

    public void acceptJoystickInputs(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier thetaSupplier,
            DoubleSupplier povSupplierDegrees) {
        mTeleopController.acceptJoystickInputs(xSupplier, ySupplier, thetaSupplier, povSupplierDegrees);
    }

    public boolean atGoal() {
        return mDriveState == DriveState.AUTO_ALIGN && mAutoAlignController.atGoal();
    }

    public boolean notAtGoal() {
        return mDriveState != DriveState.AUTO_ALIGN || !mAutoAlignController.atGoal();
    }

    public boolean getDriveToPoseTolerance() {
        return mAutoAlignController.atGoal();
    }

    public Command waitUnitllAutoAlignFinishes() {
        return new WaitUntilCommand(() -> mAutoAlignTimeout.calculate(mAutoAlignController.atGoal()));
    }

    public BooleanSupplier waitUnitllAutoAlignFinishesSupplier() {
        return () -> mAutoAlignController.atGoal();
    }
}
