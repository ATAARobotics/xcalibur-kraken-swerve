package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private double elevatorSpeed = 0;

    public CANSparkFlex leftClimbMotor = new CANSparkFlex(15, MotorType.kBrushless);
    public CANSparkFlex rightClimbMotor = new CANSparkFlex(16, MotorType.kBrushless); // Put IDs in Constants.java

    public ElevatorSubsystem() {

        leftClimbMotor.setInverted(false);
        rightClimbMotor.setInverted(true);

        leftClimbMotor.setIdleMode(IdleMode.kBrake);
        rightClimbMotor.setIdleMode(IdleMode.kBrake);


    }

    @Override
    public void periodic() {
        leftClimbMotor.set(elevatorSpeed);
        rightClimbMotor.set(elevatorSpeed); // Switch to enum once encoder works

    }

    public void elevatorUp() {
        elevatorSpeed = 0.3;
    }

    public void elevatorDown() {
        elevatorSpeed = -0.3; 
    }


}
