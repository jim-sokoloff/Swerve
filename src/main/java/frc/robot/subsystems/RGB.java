package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class RGB extends SubsystemBase {
    private AddressableLED m_led;
    public AddressableLEDBuffer LEDs;

    public enum LEDColor {
        Default(0, 0, 0),
        Blue(0, 0, 88),
        Red(88, 0, 0),
        Green(0, 128, 0),
        Gray(128, 128, 128);

        public final int red;
        public final int green;
        public final int blue;

        private LEDColor(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    public enum LEDMode {
        Loop,
        Solid;
    }

    private LEDColor m_ledColor = LEDColor.Default;
    private LEDMode m_ledMode = LEDMode.Solid;

    public RGB(int port) {
        m_led = new AddressableLED(port);

        // Reuse buffer
        // Default to a length of 100, start empty output
        // Length is expensive to set, so only set it once, then just update data
        LEDs = new AddressableLEDBuffer(100);
        m_led.setLength(LEDs.getLength());

        LEDs.setRGB(0, 255, 0 ,0);
        LEDs.setRGB(1, 0, 255, 0);
        LEDs.setRGB(2, 0, 0, 255);
        LEDs.setRGB(3, 255, 255, 255);

        // Set the data
        m_led.setData(LEDs);
        m_led.start();
    }

    public void setLedState(LEDColor color, LEDMode mode) {
        m_ledColor = color;
        m_ledMode = mode;
    }

    public void clearLEDs(int start, int end) {
        for (int i = start; i <= end; i++)
            LEDs.setRGB(i, 0, 0, 0);
    }

    public void clearLEDs() {
        clearLEDs(0, LEDs.getLength() - 1);
    }

    // 35 for outer only, 59 for inner and outer
    public final int kNumLEDsInRing = 59;

    @Override
    public void periodic() {
        clearLEDs();
        
        // I know this logic is different than before, but I think if you want to use LED indicators on the robot, you want to use a Solid pattern so it can be easily seen across the field
        // And you really only need the Alliance Color during startup to make sure you have selected the right alliance in the Driver Station
        switch(m_ledMode) {
            case Loop:
                loopLEDs();
                break;
            case Solid:
                solidLEDs();
                break;
        }

        // This method will be called once per scheduler run
        m_led.setData(LEDs);
    }

    private void loopLEDs() {
        double kLoopSpeed = 27 * 60.0 / kNumLEDsInRing;

        double matchTime = Timer.getMatchTime();
        double fpgaTime = Timer.getFPGATimestamp();
        int fpgaTimeInt = (int) fpgaTime;

        double loopTime = fpgaTime * kLoopSpeed;
        int loopTimeInt = (int) loopTime;

        // Looping alliance color
        for (int i = 0; i < RobotContainer.AllianceStationNumber(); i++) {
            LEDs.setRGB((loopTimeInt + i * 1) % kNumLEDsInRing, m_ledColor.red, m_ledColor.green, m_ledColor.blue);
        }

        SmartDashboard.putNumber("Match Time", matchTime);
        SmartDashboard.putNumber("FPGA Time", fpgaTime);
    }

    private void solidLEDs() {
        for (int i = 0; i < kNumLEDsInRing; i++) {
            LEDs.setRGB(i, m_ledColor.red, m_ledColor.green, m_ledColor.blue);
        }
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}

