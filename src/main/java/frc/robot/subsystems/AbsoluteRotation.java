package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AbsoluteRotation extends SubsystemBase{
    double rotSpeed;
    double joyAngle;
    double robAngle;

    double rotP;
    double rotI;
    double rotD;

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

        SmartDashboard.putNumber("rotP", 0);
        SmartDashboard.putNumber("rotI", 0);
        SmartDashboard.putNumber("rotD", 0);

        controller = new PIDController(rotP, rotI, rotD);

    }

    @Override
    public void periodic() {
        rotP = SmartDashboard.getNumber("rotP", 0);
        rotI = SmartDashboard.getNumber("rotI", 0);
        rotD = SmartDashboard.getNumber("rotD", 0);
        
        System.out.println(rotP);

        controller.setPID(rotP, rotI, rotD);

        joyAngle = Math.atan2(joyRightY.getAsDouble(), joyRightX.getAsDouble());
        robAngle = robotAngle.getAsDouble();

        controller.enableContinuousInput(-Math.PI, Math.PI);

        controller.setSetpoint(joyAngle);



        SmartDashboard.putNumber("RotValue", rotSpeed);
        SmartDashboard.putNumber("SetPoint", controller.getSetpoint());
        SmartDashboard.putBoolean("At Setpoint", controller.atSetpoint());
        SmartDashboard.putNumber("Robot Rot", robAngle);




        // rotSpeed = MathUtil.clamp(controller.calculate(robAngle), Constants.lowBound, Constants.MaxAngularSpeed);
        rotSpeed = controller.calculate(robAngle);


        System.out.println(rotSpeed);

    }

    public double rotationSpeed() {
        return rotSpeed;
    }
    
}
