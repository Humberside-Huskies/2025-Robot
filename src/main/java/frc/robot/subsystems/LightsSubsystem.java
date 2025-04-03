package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {

    private final AddressableLED           ledString       = new AddressableLED(LightsConstants.LED_STRING_PWM_PORT);
    private final AddressableLEDBuffer     ledBuffer       = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

    private final AddressableLEDBufferView leftSpeedBuffer = new AddressableLEDBufferView(ledBuffer, 0,
        LightsConstants.LED_STRING_LENGTH / 2);
    // private final AddressableLEDBufferView rightSpeedBuffer = new AddressableLEDBufferView(ledBuffer,
    // LightsConstants.LED_STRING_LENGTH / 2, LightsConstants.LED_STRING_LENGTH).reversed();


    public LightsSubsystem() {

        // Start the LED string
        ledString.setLength(LightsConstants.LED_STRING_LENGTH);
        ledString.start();

        setLEDColor(255, 255, 0);
    }

    private void setLEDColor(int red, int green, int blue) {

        for (int i = 0; i < 11; i++) {
            leftSpeedBuffer.setRGB(i, red, green, blue);
            // rightSpeedBuffer.setRGB(i, red, green, blue);
            // ledBuffer.setRGB(i, red, green, blue);
        }

        ledString.setData(ledBuffer);
    }

    @Override
    public void periodic() {

    }
}