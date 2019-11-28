package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
@TeleOp(name="OdometryTest", group = "TeleOp")
public class OdometryTest extends OpMode {
//    public DcMotor fr = null;
//    public DcMotor fl = null;
    public DcMotor br = null;
//    public DcMotor bl = null;

    HardwareMap hwMap = null;

    @Override
    public void init() {
        hwMap = hardwareMap;

//        fl = hwMap.get(DcMotor.class, "fl");
//        bl = hwMap.get(DcMotor.class, "bl");
//        fr = hwMap.get(DcMotor.class, "fr");
        br = hwMap.get(DcMotor.class, "br");

//        fl.setDirection(DcMotor.Direction.REVERSE);
//        bl.setDirection(DcMotor.Direction.REVERSE);
//
//        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

//        fl.setPower(0);
//        bl.setPower(0);
//        fr.setPower(0);
        br.setPower(0);
    }

    @Override
    public void loop() {

        br.setTargetPosition(1000);
//        telemetry.addData("middle: ", fl.getCurrentPosition());
        telemetry.addData("right: ", br.getCurrentPosition());

//        telemetry.addData("left: ", bl.getCurrentPosition());
    }

}
