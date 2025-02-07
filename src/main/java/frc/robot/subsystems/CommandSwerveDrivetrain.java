package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private SwerveDrivePoseEstimator PoseEstimator;


    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private DoubleTopic dblTopic;
    private DoubleSubscriber dblSub;

    private final NetworkTable table = inst.getTable("Pose");
    private final NetworkTable topicTable = inst.getTable("POSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSs");

    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        setPathPlanner();

        PoseEstimator = new SwerveDrivePoseEstimator(
            Constants.swerveKinematics,
            this.getPigeon2().getRotation2d(),
            TunerConstants.mSwerveModulePositions,
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
        );

        dblTopic = topicTable.getDoubleTopic("POSSSSSX");
        dblSub = dblTopic.subscribe(0.0);

    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        setPathPlanner();

        PoseEstimator = new SwerveDrivePoseEstimator(
            Constants.swerveKinematics,
            this.getPigeon2().getRotation2d(),
            TunerConstants.mSwerveModulePositions,
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0))
        );

        dblTopic = topicTable.getDoubleTopic("POSSSSSX");
        dblSub = dblTopic.subscribe(0.0);


    }

    private void setPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::seedFieldRelative, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Pose2d getPose() {
        return this.PoseEstimator.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return PoseEstimator;
    }

    private ChassisSpeeds getSpeeds() {
        return this.getState().speeds;
    }

    private void setSpeeds(ChassisSpeeds speeds) {
        SwerveRequest request = new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds);
        this.setControl(request);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {

        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }

        try {
            NetworkTable pose = NetworkTableInstance.getDefault().getTable("POSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSs");
            double poseX = pose.getEntry("POSSSSSX").getDouble(0);
            double poseY = pose.getEntry("POSSSSSY").getDouble(0);
            Rotation2d poseR = Rotation2d.fromRadians(pose.getEntry("ROTTTTTY").getDouble(0));
            double timeStamp = pose.getEntry("LATENSEEEEE").getDouble(0) / (1000000000); // one bil nano -> sec
            SmartDashboard.putBoolean("Limelight Status", true);
            Pose2d visionBotPose = new Pose2d(poseX, poseY, poseR);

            TimestampedDouble latency =  dblSub.getAtomic();

            double latencyDouble = (double) latency.serverTime / 1000000;

            // distance from current pose to vision estimated pose
            // double poseDifference = this.getPose().getTranslation().getDistance(visionBotPose.getTranslation());

            if (Math.abs(poseX) >= 0.001) {
                // double xyStds;
                // double degStds;
                // multiple targets detected
                // if (pose.getEntry() >= 2) {
                //     if (!DriverStation.isEnabled()) {
                //         this.getPigeon2().setYaw(poseR.getDegrees());
                //     }
                //     xyStds = 0.5;
                //     degStds = 6;
                // }
                // // 1 target with large area and close to estimated pose
                // else if (pose[9] > 0.8 && poseDifference < 0.5) {
                //     xyStds = 1.0;
                //     degStds = 12;
                // }
                // // 1 target farther away and estimated pose is close
                // else if (pose[9] > 0.1 && poseDifference < 0.3) {
                //     xyStds = 2.0;
                //     degStds = 30;
                // }
                // // conditions don't match to add a vision measurement
                // else {
                //     return;
                // }

                // this.addVisionMeasurement(visionBotPose, timeStamp
                //         // , VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds))
                //         );
                PoseEstimator.update(this.getPigeon2().getRotation2d(), TunerConstants.mSwerveModulePositions);
                PoseEstimator.addVisionMeasurement(visionBotPose, latencyDouble);

                SmartDashboard.putNumber("Latency thing", latencyDouble);
                SmartDashboard.putNumber("RIO Latency thing", Timer.getFPGATimestamp());
                SmartDashboard.putNumber("Difference in Latency thing", latencyDouble - Timer.getFPGATimestamp());

                
            }

        } catch (Exception e) {
            DriverStation.reportError("LIMELIGHT FAIL: RESTART ROBOT CODE", e.getStackTrace());
            SmartDashboard.putBoolean("Limelight Status", false);
        }

        SmartDashboard.putNumber("PoseEstimator X", PoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("PoseEstimator Y", PoseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("PoseEstimator ROT", PoseEstimator.getEstimatedPosition().getRotation().getDegrees());

        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {
            PoseEstimator.getEstimatedPosition().getX(),
            PoseEstimator.getEstimatedPosition().getY(),
            PoseEstimator.getEstimatedPosition().getRotation().getDegrees()
        });

        SmartDashboard.putNumber("Pose Estimator ", this.getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("Get Yaw ", this.getPigeon2().getYaw().getValueAsDouble());

    }
}
