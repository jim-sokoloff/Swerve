package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.TeleopSwerve;

public class RGB extends SubsystemBase {
    private AddressableLED m_led;
    public AddressableLEDBuffer LEDs;

    private static int m_LEDLoopCount;

    private final static int kTotalLEDs = 100;
    private static Random m_random = new Random();
    private final static int kIntroTime = 12;

    public RGB(int port) {
        m_led = new AddressableLED(port);

        // Reuse buffer
        // Default to a length of 100, start empty output
        // Length is expensive to set, so only set it once, then just update data
        LEDs = new AddressableLEDBuffer(kTotalLEDs);
        m_led.setLength(LEDs.getLength());

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
    //public final int kNumLEDsInRing = 59;

    @Override
    public void periodic() {
        m_LEDLoopCount++;
        if (m_LEDLoopCount < 5)
            return;
        m_LEDLoopCount = 0;

        double fpgaTime = Timer.getFPGATimestamp();
        int fpgaTimeInt = (int) fpgaTime;

        if (fpgaTimeInt < kIntroTime) {
            DoIntroAnimation2(fpgaTime);
            return;
        }
        // clearLEDs();
        
        double kLoopSpeed = 7;

        double matchTime = Timer.getMatchTime();

        double loopTime = fpgaTime * kLoopSpeed;
        int loopTimeInt = (int) loopTime;

        final int NINE_CIRCLE_START = 0;
        final int NINE_CIRCLE_END = NINE_CIRCLE_START + 24 - 1;
        final int NINE_STROKE_START = NINE_CIRCLE_END + 1;
        final int NINE_STROKE_END = NINE_STROKE_START + 10 - 1;
        final int SEVEN_TOP_START = NINE_STROKE_END + 1;
        final int SEVEN_TOP_END = SEVEN_TOP_START + 10 - 1;
        final int SEVEN_STROKE_START = SEVEN_TOP_END + 1;
        final int SEVEN_STROKE_END = SEVEN_STROKE_START + 18 - 1;
        
        // // 9 circle
        // for (int j = NINE_CIRCLE_START; j <= NINE_CIRCLE_END; j++) {
        //     LEDs.setRGB(j, 100, 0, 0);
        // }
        // // 9 stroke
        // for (int j = NINE_STROKE_START; j <= NINE_STROKE_END; j++) {
        //     LEDs.setRGB(j, 100, 100, 0);
        // }
        // // 7 top
        // for (int j = SEVEN_TOP_START; j <= SEVEN_TOP_END; j++) {
        //     LEDs.setRGB(j, 0, 0, 100);
        // }
        // // 7 stroke
        // for (int j = SEVEN_STROKE_START; j <= SEVEN_STROKE_END; j++) {
        //     LEDs.setRGB(j, 0, 100, 0);
        // }

        // Full Rainbow with wipe in
        for (int j = NINE_CIRCLE_START; j <= SEVEN_STROKE_END; j++) {
            if (j >= (fpgaTime - kIntroTime) * 20)
                break;
            LEDs.setHSV(j, (int) (j * 2.95), 255, 80);
        }

        // Translate speed on stroke of NINE
        if (TeleopSwerve.translationVal > 0) {
            for (int j = 0; j <= NINE_STROKE_END - NINE_STROKE_START; j++) {
                if (j < (TeleopSwerve.translationVal * 10))
                    LEDs.setRGB(NINE_STROKE_END - j, 0, 128, 0);
            }
        }
        if (TeleopSwerve.translationVal < 0) {
            for (int j = 0; j <= NINE_STROKE_END - NINE_STROKE_START; j++) {
                if (j < (TeleopSwerve.translationVal * -10))
                    LEDs.setRGB(NINE_STROKE_START + j, 128, 0, 0);
            }
        }

        // Strafe speed on top of SEVEN
        if (TeleopSwerve.strafeVal > 0) {
            for (int j = 0; j <= SEVEN_TOP_END - SEVEN_TOP_START; j++) {
                if (j < (TeleopSwerve.strafeVal * 10))
                    LEDs.setRGB(SEVEN_TOP_END - j, 128, 0, 0);
            }
        }
        if (TeleopSwerve.strafeVal < 0) {
            for (int j = 0; j <= SEVEN_TOP_END - SEVEN_TOP_START; j++) {
                if (j < (TeleopSwerve.strafeVal * -10))
                    LEDs.setRGB(SEVEN_TOP_START + j, 0, 128, 0);
            }
        }

        // Rotation speed in circle
        if (TeleopSwerve.rotationVal > 0) {
            for (int j = 0; j <= NINE_CIRCLE_END - NINE_CIRCLE_START; j++) {
                if (j < (TeleopSwerve.rotationVal * 100))
                    LEDs.setRGB(NINE_CIRCLE_END - j, 128, 0, 0);
            }
        }
        if (TeleopSwerve.rotationVal < 0) {
            for (int j = 0; j <= NINE_CIRCLE_END - NINE_CIRCLE_START; j++) {
                if (j < (TeleopSwerve.rotationVal * -100))
                    LEDs.setRGB(NINE_CIRCLE_START + j, 80, 0, 60);
            }
        }


        int kNumLEDsInTotal = SEVEN_STROKE_END + 1;

        // Looping alliance color
        for (int i = 0; i < RobotContainer.AllianceStationNumber(); i++) {
            if (RobotContainer.AllianceIsBlue())
                LEDs.setRGB((loopTimeInt + i * 1) % kNumLEDsInTotal, 0, 0, 88);
            else
                LEDs.setRGB((loopTimeInt + i * 1) % kNumLEDsInTotal, 88, 0, 0);
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

    private void DoIntroAnimation1(double fpgaTime) {
        int LED = 0;
        
        for (int j = 0; j < 10; j++) {
            LED = m_random.nextInt(kTotalLEDs);
            LEDs.setHSV(LED, m_random.nextInt(200), m_random.nextInt(20) + 220, m_random.nextInt((int) fpgaTime * 10 + j * 2));
            LED = m_random.nextInt(kTotalLEDs);
            int greyValue = (int) (fpgaTime * 1) + j;
            LEDs.setRGB(LED, greyValue, greyValue, greyValue);
        }
        LED = m_random.nextInt(kTotalLEDs);
        LEDs.setRGB(LED, 200, 200, 200);
        LED = m_random.nextInt(kTotalLEDs);
        LEDs.setRGB(LED, 0, 0, 0);
        


        SmartDashboard.putNumber("FPGA Time", fpgaTime);
    
        // This method will be called once per scheduler run
        m_led.setData(LEDs);
    }

    private void DoIntroAnimation2(double fpgaTime) {
        int LED = 0;
        
        if (fpgaTime < 7)
            clearLEDs();
        for (int j = 0; j < 5; j++) {
            LED = m_random.nextInt(kTotalLEDs);
            LEDs.setHSV(LED, m_random.nextInt(180), m_random.nextInt(20) + 220, m_random.nextInt(50 + 200));
            LED = m_random.nextInt(kTotalLEDs);
            int greyValue = 255;
            LEDs.setRGB(LED, greyValue, greyValue, greyValue);
        }

        SmartDashboard.putNumber("FPGA Time", fpgaTime);
    
        // This method will be called once per scheduler run
        m_led.setData(LEDs);
    }

    
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
}

