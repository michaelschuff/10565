package org.firstinspires.ftc.teamcode.Autonomous.IntakeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous(group = "Auto")
public class BlueIntakeEverythingParkBridge extends LinearOpMode {
    private SampleMecanumDriveREVOptimized drive;

    //phone offsets
    private double pxOffset = 0, pyOffset = 0, pzOffset = 0;

    private double tileWidth = 23.5, startingAngle = Math.toRadians(-90), startingX = -3 * tileWidth + 8.625 + 0.375, startingY = -tileWidth - 6.25 - pyOffset;
    @Override
    public void runOpMode() {

    }
}
