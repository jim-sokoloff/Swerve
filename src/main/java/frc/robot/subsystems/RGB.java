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
        
        double kLoopSpeed = 27 * 60.0 / kNumLEDsInRing;

        double matchTime = Timer.getMatchTime();
        double fpgaTime = Timer.getFPGATimestamp();
        int fpgaTimeInt = (int) fpgaTime;

        double loopTime = fpgaTime * kLoopSpeed;
        int loopTimeInt = (int) loopTime;

        // Looping alliance color
        for (int i = 0; i < RobotContainer.AllianceStationNumber(); i++) {
            if (RobotContainer.AllianceIsBlue())
                LEDs.setRGB((loopTimeInt + i * 1) % kNumLEDsInRing, 0, 0, 88);
            else
                LEDs.setRGB((loopTimeInt + i * 1) % kNumLEDsInRing, 88, 0, 0);
        }

        // Original red/green toggle
        boolean even = fpgaTimeInt % 2 == 0;
        //LEDs.setRGB(10, even ? 128 : 0, even ? 0 : 128, 0);

        // If zeroGyro is being commanded
        if (RobotContainer.zeroGyro.getAsBoolean()) {
            LEDs.setRGB(11, 0, 128, 0);
        }

         // If launchNote is being commanded
        if (RobotContainer.launchNote.getAsBoolean()) {
            LEDs.setRGB(12, 128, 128, 128);
        }

        SmartDashboard.putNumber("Match Time", matchTime);
        SmartDashboard.putNumber("FPGA Time", fpgaTime);
    
        // This method will be called once per scheduler run
        m_led.setData(LEDs);
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}

