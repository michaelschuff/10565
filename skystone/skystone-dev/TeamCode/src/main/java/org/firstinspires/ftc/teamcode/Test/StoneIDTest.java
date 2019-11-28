package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Auto.VuforiaLib_SkyStone;

@Config
@Autonomous (name="StoneIDTest", group = "Test")
public class StoneIDTest extends LinearOpMode {

    VuforiaLib_SkyStone mvLib = new VuforiaLib_SkyStone();
    @Override
    public void runOpMode(){
        mvLib.init(this, "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc");
        mvLib.start();


        waitForStart();



        while(opModeIsActive()) {
            mvLib.loop(true);
            try {
                telemetry.addData("X: ", mvLib.getFieldPosition().get(0));
            } catch (Exception e) {
                telemetry.addData("could not get X position", "");
            }
            telemetry.update();

            if (isStopRequested()) return;

        }

    }
}
