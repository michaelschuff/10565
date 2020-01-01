package org.firstinspires.ftc.teamcode.Tests.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(group = "HardwareTests")
public class ServoTest extends LinearOpMode {
    private Servo TestServo;

    public static double value = 0;
    public static String ConfigName = "testServo";
    String currentName = "testServo";

    @Override
    public void runOpMode() {
        currentName = ConfigName;
        TestServo = hardwareMap.get(Servo.class, ConfigName);

        if (isStopRequested()) return;

        while(!isStopRequested()) {
            TestServo.setPosition(value);
            if (currentName != ConfigName) {
                currentName = ConfigName;
                TestServo = hardwareMap.get(Servo.class, ConfigName);
            }
        }
    }
}