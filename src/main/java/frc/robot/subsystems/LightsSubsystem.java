package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {

    private final AddressableLED       ledString      = new AddressableLED(LightsConstants.LED_STRING_PWM_PORT);
    private final AddressableLEDBuffer ledBuffer      = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

    private boolean                    changeDetected = true;

    private boolean                    defaultState   = true;
    private boolean                    algaeDetected  = false;

    private Timer                      philipTimer    = new Timer();
    private final AddressableLEDBuffer philipBuffer   = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

    public LightsSubsystem() {

        // Start the LED string
        ledString.setLength(LightsConstants.LED_STRING_LENGTH);
        ledString.start();

        philipTimer.restart();
    }

    public void setLEDPhilip() {

        // Switch the pattern every 0.3 seconds
        if (!philipTimer.hasElapsed(0.3)) {
            return;
        }
        philipTimer.restart();

        // Make a new pattern
        for (int index = 0; index < this.ledBuffer.getLength(); index++) {
            int hue = (int) Math.floor(Math.random() * 255);
            philipBuffer.setHSV(index, hue, 190, 250);
        }

        ledString.setData(philipBuffer);
    }

    public void setAlgaeDetected(boolean algaeDetected) {

        // Determine if this is a new state for the algae
        if (algaeDetected != this.algaeDetected) {
            this.algaeDetected = algaeDetected;
            changeDetected     = true;
        }
    }

    @Override
    public void periodic() {

        // Only write the buffer if a change is detected
        // Don't rewrite it every loop.
        if (changeDetected) {

            defaultState = false;

            if (algaeDetected) {
                LEDPattern.solid(Color.kAzure).applyTo(ledBuffer);
            }
            // FIXME: add more patterns here
            else {
                // Default state
                defaultState = true;
            }

            changeDetected = false;

            if (!defaultState) {
                ledString.setData(ledBuffer);
            }
        }

        if (defaultState) {
            setLEDPhilip();
        }
    }

}