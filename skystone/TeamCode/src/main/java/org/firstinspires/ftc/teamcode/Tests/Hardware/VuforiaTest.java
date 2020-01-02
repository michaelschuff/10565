package org.firstinspires.ftc.teamcode.Tests.Hardware;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

@Autonomous(group = "HardwareTests")
public class VuforiaTest extends LinearOpMode {
    VuforiaLib_Skystone camera;
    VectorF fieldPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new VuforiaLib_Skystone();
        camera.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        camera.start();

        waitForStart();

        while(opModeIsActive()) {
            camera.loop(true);
//            try {
//                fieldPosition = camera.getFieldPosition();
//                telemetry.addData("PosX", fieldPosition.get(0) / 25.4);
//                telemetry.addData("PosY", fieldPosition.get(1) / 25.4);
//                telemetry.addData("PosZ", fieldPosition.get(2) / 25.4);
//            } catch (Exception e) {
//                telemetry.addData("PosX", null);
//                telemetry.addData("PosY", null);
//                telemetry.addData("PosZ", null);
//            }
//            telemetry.update();
        }
    }
}
