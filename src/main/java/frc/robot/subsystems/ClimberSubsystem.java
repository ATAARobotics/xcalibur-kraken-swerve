package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    public CANSparkFlex leftClimb;
    public CANSparkFlex rightClimb;

    private double speed = 0;

    public ClimberSubsystem() {
        leftClimb = new CANSparkFlex(13, MotorType.kBrushless);
        rightClimb = new CANSparkFlex(14, MotorType.kBrushless);

    }

    @Override
    public void periodic() {
        leftClimb.set(-speed);
        rightClimb.set(speed);

    }

    public void runClimb() {
        speed = 0.4;
    }

    public void stopClimb() {
        speed = 0.0;
    }

    public void reverseClimb() {
        speed = -0.4;
    }

}
