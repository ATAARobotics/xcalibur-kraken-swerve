package frc.robot;

import java.lang.Math;
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
    
}
