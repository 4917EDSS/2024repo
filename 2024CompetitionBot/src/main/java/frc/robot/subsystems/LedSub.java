// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PwmIds;


public class LedSub extends SubsystemBase {
  private static Logger m_logger = Logger.getLogger(LedSub.class.getName());
  private final ArduinoSub m_arduinoSub;

  // Constants
  private final static int kLedStripLength = 33;
  //private final ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("LedSubTab");

  private static int[][] m_ledColourBuffer = new int[kLedStripLength][3];
  private boolean m_newColoursAvailable = false;
  private boolean m_isFlashing; //true if flash is on (game piece gets loaded)
  private long m_time; //time of when the flash starts
  private int m_ledblinktimes = 0; // Number of times the led should blink when flashing

  public enum LedZones {
    // The LED string start at the top left and is split up in a big U shape as follows:
    // - 9 triplets on the left mast vertical hockey stick, facing the front
    // - 9 triplets on the left mast angled hockey stick, facing the back
    // - 6 triplets on the left half of the horizontal hockey stick, facing the back
    // - The same 6, 9, 9 mirrored on the right side of the robot

    // Normal operation zones
    ALL(0, kLedStripLength - 1, false), // Set all the LEDs
    GAME_PIECE(0, 16, true), // Indicates if in or not
    ARM_POSITION(21, 21, true), // Indicates if the arm is in speaker or AMP position from passing through (red) or not (green)
    VISION(20, 20, true), // Indicates if the vision sees a traget (green) or not (red)

    // Disabled diagnostic zones
    //DIAG_DRIVE_ENC(14, 14, false), //
    DIAG_CLIMBL_LIMIT(7, 7, false), //
    DIAG_CLIMBR_LIMIT(8, 8, false), //
    DIAG_SHOOTERFWD_LIMIT(9, 9, false), //
    DIAG_SHOOTERREV_LIMIT(10, 10, false), //
    DIAG_PIVOT_ENC(11, 11, false), //
    DIAG_FL_STEERING_ENC(0, 6, false), //
    DIAG_FR_STEERING_ENC(12, 18, false), //
    DIAG_BR_STEERING_ENC(19, 25, false), //
    DIAG_BL_STEERING_ENC(26, 32, false); //
    //DIAG_INTAKE_ENC(20, 20, false), //
    //DIAG_INTAKE_SENSOR(21, 21, false), DIAG_NOTE_INSIDE(0, 3, false), // Indicates if a note is inside storage. Was originally called Limit Switches Hit
    //DIAG_SHOOTER_POSITION(4, kLedStripLength - 1, false); //indicates shooter position

    public final int start;
    public final int end;
    public final boolean mirror;

    LedZones(int start, int end, boolean mirror) {
      this.start = start;
      this.end = end;
      this.mirror = mirror;
    }
  }

  public enum LedColour {
    YELLOW(128, 100, 0), //
    PURPLE(80, 20, 60), //
    RED(128, 0, 0), //
    GREEN(0, 128, 0), //
    START_GREEN(45, 128, 0), // 
    BLUE(10, 0, 128), //
    WHITE(128, 128, 128),
    //
    ORANGE(128, 20, 0);

    public final int red, blue, green;

    LedColour(int red, int green, int blue) {
      int r = red;
      int g = green;
      int b = blue;

      if(r < 0) {
        r = 0;
      }
      if(r > 255) {
        r = 255;
      }
      if(g < 0) {
        g = 0;
      }
      if(g > 255) {
        g = 255;
      }
      if(b < 0) {
        b = 0;
      }
      if(b > 255) {
        b = 255;
      }

      this.red = r;
      this.green = g;
      this.blue = b;
    }
  }

  // Hardware setup.
  // WARNING!!! We can only have one LED strip.  roboRIO does not support two AddressableLED objects.
  AddressableLED m_ledStrip = new AddressableLED(PwmIds.kLedStripPwmPort);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(kLedStripLength);

  /** Creates a new LedSub. */
  public LedSub(ArduinoSub arduinoSub) {
    m_arduinoSub = arduinoSub;
    m_ledStrip.setLength(m_ledBuffer.getLength());
    m_ledStrip.setData(m_ledBuffer);
    m_ledStrip.start();

    init();
  }

  /**
   * Use this method to reset all of the hardware and states to safe starting values
   */
  public void init() {
    m_logger.info("Initializing LedSub");
    setZoneColour(LedZones.ALL, LedColour.START_GREEN);
  }

  @Override
  public void periodic() {
    if(m_newColoursAvailable) {
      m_ledStrip.setData(m_ledBuffer);
      m_newColoursAvailable = false;
    }

    if(m_isFlashing) {
      long timeSinceIntakeLoaded = RobotController.getFPGATime() - m_time;

      if(timeSinceIntakeLoaded < 250000) { // Led ON time
        setZoneColour(LedZones.ALL, LedColour.PURPLE);
      }
      if(250000 <= timeSinceIntakeLoaded && timeSinceIntakeLoaded <= 500000) {
        setZoneColour(LedZones.ALL, LedColour.WHITE);
      }
      if(timeSinceIntakeLoaded > 500000) {
        m_time = RobotController.getFPGATime();
        m_ledblinktimes++;
      }
      if(m_ledblinktimes >= 3) {//when it is done flashing
        m_isFlashing = false;
        m_ledblinktimes = 0;
        setZoneColour(LedZones.GAME_PIECE, LedColour.PURPLE);
      }
    }
  }

  // This method will be called once per scheduler run
  public void flashLEDs() {
    m_isFlashing = true;
    m_time = RobotController.getFPGATime(); // The time when the flashing will begin
  }

  /**
   * Set all the LEDS in the specified zone to the specified colour. Define more colours if needed.
   */
  public void setZoneColour(LedZones zone, LedColour ledColour) {
    setZoneRGB(zone, ledColour.red, ledColour.green, ledColour.blue);
    //adding local colors because led colors set here are final. Maybe they shouldn't be?
    int r = ledColour.red;
    int g = ledColour.green;
    int b = ledColour.blue;

    if(zone == LedZones.ALL) {
      if(ledColour.red + ledColour.blue + ledColour.blue > 510) { //might already be divided in arduino sub
        r = ledColour.red / 2; //divide them by 2 because it draws too much voltage from my understanding
        g = ledColour.green / 2;
        b = ledColour.blue / 2;
      }
      m_arduinoSub.updateLED(r, g, b);
    }

    // TODO: If zone is ALL, also set the Arduino board LEDs to this colour (but don't let the R + G + B value exceed 510)
    // Something like 
    // if(sum-of-LED-colours > 510) {
    //   Divide each colour value by two
    // }
    // And then
    // m_arduinoSub.updateLED(ledColour.red, ledColour.green, ledColour.blue);
  }

  private void setBuffer(int position, int r, int g, int b) {
    if(m_ledColourBuffer[position][0] == r && m_ledColourBuffer[position][1] == g
        && m_ledColourBuffer[position][2] == b) {
      return;
    }
    m_ledBuffer.setRGB(position, r, b, g); // String takes values in this order
    m_ledColourBuffer[position][0] = r;
    m_ledColourBuffer[position][1] = g;
    m_ledColourBuffer[position][2] = b;
    m_newColoursAvailable = true;
  }

  /**
   * Set all the LEDs in the specified zone to the specified RGB value. Recomment that you use setZoneColour instead.
   */
  public void setZoneRGB(LedZones zone, int r, int g, int b) {
    r = MathUtil.clamp(r, 0, 255);
    g = MathUtil.clamp(g, 0, 255);
    b = MathUtil.clamp(b, 0, 255);

    for(int i = zone.start; i <= zone.end; i++) {
      setBuffer(i, r, b, g);
    }

    if(zone.mirror) {
      // Set the same LEDs on the other half of the string (count from the end instead of the start)
      int start = kLedStripLength - zone.end - 1;
      int end = kLedStripLength - zone.start - 1;
      for(int i = start; i <= end; i++) {
        setBuffer(i, r, b, g);
      }
    }

  }
}
