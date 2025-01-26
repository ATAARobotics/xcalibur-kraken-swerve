package frc.robot;

import java.lang.Math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class Constants {

    // PID Controller variables for ABS Heading
    public static double rotKP = 0.2;
    public static double rotKI = 0;
    public static double rotKD = 0;

    // Clamp value for ABS Heading
    public static final double lowBound = Math.PI / 180;

    // Max Speeds
    public static double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    public static double MaxAngularSpeed = 3 * Math.PI;

    public static final double swerveWidth = Units.inchesToMeters(18.5);
    public static final double swerveLength = Units.inchesToMeters(22.5);

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(-swerveLength / 2.0, -swerveWidth / 2.0),
        new Translation2d(-swerveLength / 2.0, swerveWidth / 2.0),
        new Translation2d(swerveLength / 2.0, -swerveWidth / 2.0),
        new Translation2d(swerveLength  / 2.0, swerveWidth / 2.0)
    );
    
}
