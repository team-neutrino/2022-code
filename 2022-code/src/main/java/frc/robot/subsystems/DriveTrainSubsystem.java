package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorCON;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrainSubsystem extends SubsystemBase {
private CANSparkMax m_rightmotor1 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_ONE,MotorType.kBrushless);
private CANSparkMax m_rightmotor2 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_TWO, MotorType.kBrushless);
private CANSparkMax m_leftmotor1 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_THREE, MotorType.kBrushless);
private CANSparkMax m_leftmotor2 = new CANSparkMax(MotorCON.MOTOR_CONTROLLER_FOUR, MotorType.kBrushless);
private RelativeEncoder m_leftencoder1;
private RelativeEncoder m_rightencoder;

public  void DriveTrainSubsystem() {
 m_leftmotor1.setInverted(true);
 m_leftmotor2.setInverted(true);
 
}
public void drivePeriodic(double raxis, double laxis) {


    m_rightmotor1.set(raxis);
    m_rightmotor2.set(raxis);
    m_leftmotor1.set(laxis);
    m_leftmotor2.set(laxis);
    }
}

