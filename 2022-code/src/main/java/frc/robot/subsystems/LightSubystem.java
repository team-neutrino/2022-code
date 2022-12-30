package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LightSubystem extends SubsystemBase {
AddressableLED m_led = new AddressableLED(9);
AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

public LightSubystem() {

m_led.setData(m_ledBuffer);
m_led.start();

}
public void red() {
    for (int i = 0; i  < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 0, 0);
    }
m_led.setData(m_ledBuffer);
}
@Override
public void periodic(){

}
}
