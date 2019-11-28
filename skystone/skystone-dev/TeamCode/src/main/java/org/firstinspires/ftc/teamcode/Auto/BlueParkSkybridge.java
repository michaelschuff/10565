package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class BlueParkSkybridge extends LinearOpMode {

    public DcMotorEx fl, bl, br, fr;

//    public int lastLeftEncoder, lastRightEncoder, lastMiddleEncoder;
//
//    double[] samplingVals = {.19, .55};
//    double[] intakeVals = {.8, 1};
//    double[] outtakeVals = {.9, .5};
//    double[] foundationVals = {.35, .7};

    Servo samplingServo = null;
    Servo intakeServo = null;
    Servo outtakeServo = null;
    Servo[] foundationServos = new Servo[2];

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");

        samplingServo = hardwareMap.servo.get("samplingServo");
        intakeServo = hardwareMap.servo.get("intakeServo");
        outtakeServo = hardwareMap.servo.get("outtakeServo");
        foundationServos[0] = hardwareMap.servo.get("backFoundationServo");
        foundationServos[1] = hardwareMap.servo.get("frontFoundationServo");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

//        lastLeftEncoder = bl.getCurrentPosition();
//        lastRightEncoder = br.getCurrentPosition();
//        lastMiddleEncoder = fl.getCurrentPosition();

        waitForStart();

        fl.setPower(-1);//left
        bl.setPower(1);
        fr.setPower(1);
        br.setPower(-1);

        sleep(250);

        fl.setPower(1);//forward
        bl.setPower(1);
        fr.setPower(1);
        br.setPower(1);

        sleep(750   );

        fl.setPower(0);//stop
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);




    }
}
