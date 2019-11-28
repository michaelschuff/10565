package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Config
@Autonomous (name="Basic Auto", group = "Auto")
public class BasicAuto extends LinearOpMode {

    DcMotor[] driveMotors = new DcMotor[4];
    DcMotor[] intakeMotors = new DcMotor[2];
    DcMotor liftMotor = null;
    DcMotor armMotor = null;

    VuforiaLocalizer vuforia;
    VuforiaTrackables visionTargets;
    VuforiaTrackable skyStone;


    @Override
    public void runOpMode() throws InterruptedException {






        driveMotors[0] = hardwareMap.dcMotor.get("br");
        driveMotors[1] = hardwareMap.dcMotor.get("bl");
        driveMotors[2] = hardwareMap.dcMotor.get("fr");
        driveMotors[3] = hardwareMap.dcMotor.get("fl");

        driveMotors[0] = hardwareMap.dcMotor.get("lintake");
        driveMotors[1] = hardwareMap.dcMotor.get("rintake");

        liftMotor = hardwareMap.dcMotor.get("lift");
        armMotor = hardwareMap.dcMotor.get("arm");

        waitForStart();

        //Drive Code Goes Here

        //Start against wall adjacent to depot
        //Identify skystone
        //Drive to skystone
        //Pick up skystone
        //Reverse to wall
        //Drive to foundation
        //Drop stone
        //Park under bridge



    }

    public  void setupVuforia(){
        int cameraMoniterViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMoniterViewId);

        parameters.vuforiaLicenseKey = "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        visionTargets = this.vuforia.loadTrackablesFromAsset("Skystone");
        skyStone = visionTargets.get(0);

    }
}
