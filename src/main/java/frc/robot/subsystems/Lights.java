package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.IntPreferenceConstant;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Lights extends SubsystemBase {
  private IntPreferenceConstant numLEDs = new IntPreferenceConstant("Number Of LEDs", 93);
  private int m_state = 0;
  private int counter = 0;
  private final CANdle m_candle = new CANdle(Constants.CANDLE_ID);
  private boolean m_clearAnim = true;
  private boolean m_setAnim = true;

  private Animation m_toAnimate = null;
  private Animation m_lastAnimation = null;

  private BooleanSupplier m_driveReady;
  private BooleanSupplier m_armevatorReady;
  private BooleanSupplier m_doghouseReady;
  private BooleanSupplier m_climberReady;
  private BooleanSupplier m_visionReady;
  private BooleanSupplier m_hasCoral;
  private BooleanSupplier m_elevatorDown;
  private Supplier<String> m_autoName;

  private boolean m_colorSet = false;

  class Colors {
    int r, g, b;

    private Colors(int red, int green, int blue) {
      this.r = red;
      this.g = green;
      this.b = blue;
    }
  }

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll,
    Empty
  }

  // the best subsystems
  // do their work while disabled
  // the climber agrees

  public Lights(
      BooleanSupplier driveReady,
      BooleanSupplier armevatorReady,
      BooleanSupplier doghouseReady,
      BooleanSupplier climberReady,
      BooleanSupplier visionReady,
      BooleanSupplier hasCoral,
      BooleanSupplier elevatorDown,
      Supplier<String> autoName) {
    m_driveReady = driveReady;
    m_armevatorReady = armevatorReady;
    m_doghouseReady = doghouseReady;
    m_climberReady = climberReady;
    m_visionReady = visionReady;
    m_hasCoral = hasCoral;
    m_elevatorDown = elevatorDown;
    m_autoName = autoName;
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 1.0;
    configAll.vBatOutputMode = VBatOutputMode.On;
    m_candle.configAllSettings(configAll, 100);
    m_candle.configLEDType(LEDStripType.RGB, 300);
  }

  private Animation noteSpinLeft =
      new ColorFlowAnimation(165, 0, 255, 0, 0.2, numLEDs.getValue(), Direction.Forward);
  private Animation noteSpinRight =
      new ColorFlowAnimation(165, 0, 0, 255, 0.2, numLEDs.getValue(), Direction.Backward);
  private Animation holdingCoral =
      new ColorFlowAnimation(165, 0, 255, 0, 0.2, numLEDs.getValue(), Direction.Forward);
  private Animation intakingNote = new StrobeAnimation(165, 0, 255, 0, 0.2, numLEDs.getValue());
  private Animation setFire = new FireAnimation(1, 0.9, numLEDs.getValue(), 0.4, 0.4);
  private Animation rainBow = new RainbowAnimation(1, 0.7, numLEDs.getValue());

  public void noteSpinLeft() {
    m_setAnim = true;
    m_toAnimate = noteSpinLeft;
  }

  public void noteSpinRight() {
    m_setAnim = true;
    m_toAnimate = noteSpinRight;
  }

  public void holdingCoral() {
    m_setAnim = true;
    m_toAnimate = holdingCoral;
  }

  public void intakingNote() {
    m_setAnim = true;
    m_toAnimate = intakingNote;
  }

  public void setFire() {
    m_setAnim = true;
    m_toAnimate = setFire;
  }

  public void larsonColor(int r, int g, int b) {
    m_toAnimate = new LarsonAnimation(r, g, b, 0, 0.2, numLEDs.getValue(), BounceMode.Front, 8);
  }

  public void setLED(int r, int g, int b) {
    m_setAnim = false;
    m_clearAnim = true;
    m_candle.clearAnimation(0);
    m_candle.setLEDs(r, g, b);
  }

  public void disableLED() {
    m_setAnim = true;
  }

  public void rainbow() {
    m_setAnim = true;
    m_toAnimate = rainBow;
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {

      switch (m_state) {
        case 0:
          {
            if (!m_colorSet) {
              // blue
              larsonColor(0, 0, 255);
              m_colorSet = true;
            }
            if (m_driveReady.getAsBoolean() && counter++ > 50) {
              m_state++;
              counter = 0;
              m_colorSet = false;
            }
            break;
          }
        case 1:
          {
            if (!m_colorSet) {
              // pink
              larsonColor(165, 0, 255);
              m_colorSet = true;
            }
            if (m_armevatorReady.getAsBoolean() && counter++ > 50) {
              m_state++;
              counter = 0;
              m_colorSet = false;
            }
            break;
          }
        case 2:
          {
            if (!m_colorSet) {
              // orange
              larsonColor(255, 153,0);
              m_colorSet = true;
            }
            if (m_doghouseReady.getAsBoolean() && counter++ > 50) {
              m_state++;
              counter = 0;
              m_colorSet = false;
            }
            break;
          }
        case 3:
          {
            if (!m_colorSet) {
              // red
              larsonColor(255, 0, 0);
              m_colorSet = true;
            }
            if (m_climberReady.getAsBoolean() && counter++ > 50) {
              m_state++;
              counter = 0;
              m_colorSet = false;
            }
            break;
          }
        case 4:
          {
            if (!m_colorSet) {
              // green
              larsonColor(0, 255, 0);
              m_colorSet = true;
            }
            if (m_visionReady.getAsBoolean() && counter++ > 50) {
              m_state++;
              counter = 0;
              m_colorSet = false;
            }
            break;
          }
        case 5:
          {
            if (!m_colorSet) {
              // yellow
              larsonColor(255, 255, 0);
              m_colorSet = true;
            }
            if (true && counter++ > 50) {
              m_state++;
              counter = 0;
              m_colorSet = false;
            }
            break;
          }
        case 6:
          {
            if (!m_colorSet) {
              // light blue
              larsonColor(0, 255, 255);
              m_colorSet = true;
            }
            if (true && counter++ > 50) {
              m_state++;
              counter = 0;
              m_colorSet = false;
            }
            break;
          }
        case 7:
          {
            if (!m_colorSet) {
              // white
              larsonColor(255, 255, 255);
              m_colorSet = true;
            }
            if (!m_autoName.get().equals("None") && counter++ > 50) {
              m_state++;
              counter = 0;
              m_colorSet = false;
            }
            break;
          }
          // last case is default
        default:
          {
            if (!m_colorSet) {
              // rainbow
              rainbow();
              m_colorSet = true;
            }

            if (!(m_driveReady.getAsBoolean()
                && m_armevatorReady.getAsBoolean()
                && m_doghouseReady.getAsBoolean()
                && m_climberReady.getAsBoolean()
                && m_visionReady.getAsBoolean())) {
              m_state = 0;
              m_colorSet = false;
            }
            break;
          }
      }
    } else {
      if (m_hasCoral.getAsBoolean()) {
        holdingCoral();
      } else if (!m_elevatorDown.getAsBoolean()) {
        setFire();
      } else {
        rainbow();
      }
    }

    // if animation is equal to last one, don't clear
    if (m_toAnimate.equals(m_lastAnimation)) {

      m_clearAnim = false;
      // if animation if not equal to last one, clear animation
    } else if (!m_toAnimate.equals(m_lastAnimation) && m_lastAnimation != null) {
      m_lastAnimation = m_toAnimate;
      m_clearAnim = true;
      // for the very first time when m_lastAnimation is null, don't clear.
    } else {
      m_lastAnimation = m_toAnimate;
      m_clearAnim = false;
    }
    if (m_clearAnim) {
      m_candle.clearAnimation(0);
      m_clearAnim = false;
    }
    if (m_setAnim) {
      m_candle.animate(m_toAnimate);
    }
  }

  public InstantCommand spinLeftFactory() {
    return new InstantCommand(
        () -> {
          noteSpinLeft();
        });
  }

  public InstantCommand spinRightFactory() {
    return new InstantCommand(
        () -> {
          noteSpinRight();
        });
  }

  public InstantCommand holdNoteFactory() {
    return new InstantCommand(
        () -> {
          holdingCoral();
        });
  }

  public InstantCommand setFireFactory() {
    return new InstantCommand(
        () -> {
          setFire();
        });
  }

  public InstantCommand setLEDFactory(int r, int g, int b) {
    return new InstantCommand(() -> setLED(r, g, b), this);
  }
}
