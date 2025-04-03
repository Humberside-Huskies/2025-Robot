package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {

    private final AddressableLED           ledString        = new AddressableLED(LightsConstants.LED_STRING_PWM_PORT);
    private final AddressableLEDBuffer     ledBuffer        = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

    private final AddressableLEDBufferView leftSpeedBuffer  = new AddressableLEDBufferView(ledBuffer, 0,
        LightsConstants.LED_STRING_LENGTH / 2);
    private final AddressableLEDBufferView rightSpeedBuffer = new AddressableLEDBufferView(ledBuffer,
        LightsConstants.LED_STRING_LENGTH / 2, LightsConstants.LED_STRING_LENGTH - 1).reversed();
    private boolean                        firstTime        = true;

    public LightsSubsystem() {

        // Start the LED string
        ledString.setLength(LightsConstants.LED_STRING_LENGTH);
        ledString.start();

        // setLEDColor(255, 255, 0);
    }

    private void setLEDColor(int red, int green, int blue) {

        for (int i = 0; i < 11; i++) {
            leftSpeedBuffer.setRGB(i, red, green, blue);
            // rightSpeedBuffer.setRGB(i, red, green, blue);
            // ledBuffer.setRGB(i, red, green, blue);
        }

        ledString.setData(ledBuffer);
    }

    public void setLEDPhilip() {

        for (int index = 0; index < this.ledBuffer.getLength(); index++) {
            int hue = (int) Math.floor(Math.random() * 255);
            this.ledBuffer.setHSV(index, hue, 190, 250);

        }
        // for (int index = 0; index < this.ledBuffer.getLength(); index++) {
        // this.ledBuffer.setLED(index, randomColorShift(this.ledBuffer.getLED(index)));
        // }
        ledString.setData(this.ledBuffer);

    }

    private Color randomColorShift(Color aColor) {
        return new Color(randomShift(aColor.red), randomShift(aColor.green), randomShift(aColor.blue));
    }

    private double randomShift(double value) {
        double sign   = Math.random() >= 0.5 ? 1.0 : -1.0;
        double amount = Math.random() / 10;
        return MathUtil.clamp(value + sign * amount, 0, 1);
    }

    public boolean isAnimated() {
        return true;
    }


    @Override
    public void periodic() {
        setLEDPhilip();
    }
}