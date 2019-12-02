package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(group = "Tests")
public class ServoTesting extends LinearOpMode {
    private Servo TestServo;
    
    public static double value = 0;
    public static String ConfigName = "testServo";

    @Override
    public void runOpMode() {
        TestServo = hardwareMap.get(Servo.class, ConfigName);

        if (isStopRequested()) return;

        while(!isStopRequested()) {
            TestServo.setPosition(value);
        }
    }
}
