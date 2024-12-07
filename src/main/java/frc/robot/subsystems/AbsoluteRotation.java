package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AbsoluteRotation implements Subsystem{

    DoubleSupplier joyRightX;
    DoubleSupplier joyRightY;
    DoubleSupplier robotAngle;

    PIDController controller;

    public AbsoluteRotation(DoubleSupplier joyRightX, 
                            DoubleSupplier joyRightY, 
                            DoubleSupplier robotAngle) {
        this.joyRightX = joyRightX;
        this.joyRightY = joyRightY;
        this.robotAngle = robotAngle;

        controller = new PIDController(TunerConstants.rotKP, 
                                       TunerConstants.rotKI, 
                                       TunerConstants.rotKD);
    }

    @Override
    public void periodic() {
        double joyAngle = Math.atan2(joyRightY.getAsDouble(), joyRightX.getAsDouble());
        double robAngle = robotAngle.getAsDouble();

        
    }

    private void updateAngle() {

    }

    public double rotationSpeed() {

    }
    
}
