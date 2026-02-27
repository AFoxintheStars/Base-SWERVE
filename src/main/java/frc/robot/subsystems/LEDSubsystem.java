package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final LEDPattern m_scrollingRainbow;
    private final LEDPattern m_canAlertYellow = LEDPattern.solid(Color.kYellow);
    private final LEDPattern m_canAlertGreen = LEDPattern.solid(Color.kGreen);

    private static final int LED_LENGTH = 200;
    private static final Distance LED_SPACING = Meters.of(1.0 / 60.0);
    private static final double CAN_ALERT_HOLD_SECONDS = 2.0;
    private static final int ALERT_TOGGLE_TICKS = 5; // 5 * 20 ms = 100 ms

    private int m_prevBusOffCount = 0;
    private int m_prevTxFullCount = 0;
    private int m_prevReceiveErrorCount = 0;
    private int m_prevTransmitErrorCount = 0;
    private int m_alertTickCounter = 0;
    private boolean m_alertShowYellow = true;
    private double m_canAlertUntil = 0.0;

    public LEDSubsystem() {
        m_led = new AddressableLED(0); // PWM Port 0
        m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
        m_scrollingRainbow = LEDPattern.rainbow(255, 128)
            .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);

        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        CANStatus canStatus = RobotController.getCANStatus();
        boolean newCanError =
            canStatus.busOffCount > m_prevBusOffCount
            || canStatus.txFullCount > m_prevTxFullCount
            || canStatus.receiveErrorCount > m_prevReceiveErrorCount
            || canStatus.transmitErrorCount > m_prevTransmitErrorCount;

        m_prevBusOffCount = canStatus.busOffCount;
        m_prevTxFullCount = canStatus.txFullCount;
        m_prevReceiveErrorCount = canStatus.receiveErrorCount;
        m_prevTransmitErrorCount = canStatus.transmitErrorCount;

        if (newCanError) {
            m_canAlertUntil = Timer.getFPGATimestamp() + CAN_ALERT_HOLD_SECONDS;
        }

        if (Timer.getFPGATimestamp() < m_canAlertUntil) {
            m_alertTickCounter++;
            if (m_alertTickCounter >= ALERT_TOGGLE_TICKS) {
                m_alertTickCounter = 0;
                m_alertShowYellow = !m_alertShowYellow;
            }
            (m_alertShowYellow ? m_canAlertYellow : m_canAlertGreen).applyTo(m_ledBuffer);
        } else {
            m_scrollingRainbow.applyTo(m_ledBuffer);
        }

        m_led.setData(m_ledBuffer);
    }
}
