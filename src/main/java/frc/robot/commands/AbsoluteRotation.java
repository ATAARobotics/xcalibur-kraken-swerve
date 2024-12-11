// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import frc.robot.Constants;
// import frc.robot.generated.TunerConstants;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AbsoluteRotation extends Command{
//     double rotSpeed;
//     double joyAngle;
//     double robAngle;

//     DoubleSupplier joyRightX;
//     DoubleSupplier joyRightY;
//     DoubleSupplier robotAngle;

//     PIDController controller;

//     public AbsoluteRotation(DoubleSupplier joyRightX, 
//                             DoubleSupplier joyRightY, 
//                             DoubleSupplier robotAngle) {
//         this.joyRightX = joyRightX;
//         this.joyRightY = joyRightY;
//         this.robotAngle = robotAngle;

//         controller = new PIDController(Constants.rotKP, 
//                                        Constants.rotKI, 
//                                        Constants.rotKD);
//     }

//     @Override
//     public void execute() {
//         joyAngle = Math.atan2(joyRightY.getAsDouble(), joyRightX.getAsDouble());
//         robAngle = robotAngle.getAsDouble();

//         rotSpeed = MathUtil.clamp(controller.calculate(robAngle, joyAngle), Constants.lowBound, Constants.MaxAngularSpeed);
//     }

//     private void updateAngle() {

//     }

//     public double rotationSpeed() {
//         return rotSpeed;
//     }
    
// }
