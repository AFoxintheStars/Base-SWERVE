package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    private static final int LED_LENGTH = 60;

    // Used to shift the rainbow over time
    private int m_rainbowFirstPixelHue = 0;

    public LEDSubsystem() {
        m_led = new AddressableLED(0); // PWM Port 0
        m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * Creates a slow-moving rainbow gradient
     */
    public void rainbow() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {

            // Calculate hue for each pixel
            int hue = (m_rainbowFirstPixelHue + 
                      (i * 180 / m_ledBuffer.getLength())) % 180;

            // Full saturation, 50% brightness (value = 128)
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }

        // Slowly shift rainbow
        m_rainbowFirstPixelHue += 1;

        // Wrap around
        m_rainbowFirstPixelHue %= 180;
    }

    @Override
    public void periodic() {
        rainbow();
        m_led.setData(m_ledBuffer);
    }
}