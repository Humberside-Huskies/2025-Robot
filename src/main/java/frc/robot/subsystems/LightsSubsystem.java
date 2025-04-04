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
    private boolean                    coralDetected  = false;

    private double                     rainbowT       = 0;
    private static final Color         ALGAE_COLOR    = new Color(7, 17, 34);
    private Timer                      philipTimer    = new Timer();
    private final AddressableLEDBuffer philipBuffer   = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

    public LightsSubsystem() {

        // Start the LED string
        ledString.setLength(LightsConstants.LED_STRING_LENGTH);
        ledString.start();

        philipTimer.restart();
    }

    public void setLEDPhilip() {

        rainbowT += 1;
        // Switch the pattern every 0.3 seconds
        if (!philipTimer.hasElapsed(0.1)) {
            return;
        }
        philipTimer.restart();

        // Make a new pattern
        for (int index = 0; index < this.ledBuffer.getLength(); index++) {
            int hue = (int) Math.floor(Math.random() * 180);
            philipBuffer.setHSV(index, (int) (index * (int) Math.floor(180.0 / this.ledBuffer.getLength()) + rainbowT) % 180, 255,
                5);
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

    public void setCoralDetected(boolean coralDetected) {

        // Determine if this is a new state for the coral
        if (coralDetected != this.coralDetected) {
            this.coralDetected = coralDetected;
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
                LEDPattern.solid(ALGAE_COLOR).applyTo(ledBuffer);
            }
            else if (coralDetected) {
                LEDPattern.solid(Color.kWhite).applyTo(ledBuffer);
            }
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