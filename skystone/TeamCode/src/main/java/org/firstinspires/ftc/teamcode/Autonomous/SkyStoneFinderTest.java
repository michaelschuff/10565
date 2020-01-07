package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Autonomous.SkyStoneFinder;
import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;

public class SkyStoneFinderTest extends OpMode {

    VuforiaLib_Skystone mvLib;

    @Override
    public void init(){
        mvLib.init(this,"");
    }

    @Override
    public void loop(){
        mvLib.start();
        telemetry.addData("Skystone Position: ", SkyStoneFinder.detectSkystone(mvLib));
    }
}
