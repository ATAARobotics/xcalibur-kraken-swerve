package frc.robot.subsystems;
import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCAN extends SubsystemBase{
    // private LaserCan laserCAN = new LaserCan(30);

    public LaserCAN() {
        // SmartDashboard.putNumber("LaserCAN", laserCAN.getMeasurement().distance_mm);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("LaserCAN", laserCAN.getMeasurement().distance_mm);

    }
}
