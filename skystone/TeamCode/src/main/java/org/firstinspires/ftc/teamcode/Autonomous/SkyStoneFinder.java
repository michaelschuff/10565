package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;
import android.graphics.Bitmap;


public class SkyStoneFinder {

    static int detectSkystone(VuforiaLib_Skystone mvLib){
        Bitmap bmIn = mvLib.getBitmap(4);
        int[] inArray = new int[bmIn.getWidth() * bmIn.getHeight()];
        bmIn.getPixels(inArray, 0, bmIn.getWidth(), 0, 0, bmIn.getWidth(), bmIn.getHeight());
        int width = bmIn.getWidth();
        int stoneOneAvg = 0, stoneTwoAvg = 0, stoneThreeAvg = 0;
        int skyStone;
        for(int i = 310; i < 410; i++){
            for(int j = 400; j < 550; j++){
                stoneOneAvg += (inArray[j + i*width] >> 16) & 0xFF;
            }
        }
        stoneOneAvg /= 15000;
        for(int i = 310; i < 410; i++){
            for(int j = 700; j < 850; j++){
                stoneTwoAvg += (inArray[j + i*width] >> 16) & 0xFF;
            }
        }
        stoneTwoAvg /= 15000;
        for(int i = 310; i < 410; i++){
            for(int j = 1000; j < 1150; j++){
                stoneThreeAvg += (inArray[j + i*width] >> 16) & 0xFF;
            }
        }
        stoneThreeAvg /= 15000;
        if(stoneOneAvg < stoneTwoAvg && stoneOneAvg < stoneThreeAvg){
            skyStone = 0;
        }else if(stoneTwoAvg < stoneOneAvg && stoneTwoAvg < stoneThreeAvg){
            skyStone = 1;
        }else{
            skyStone = 2;
        }
        return skyStone;
    }

}
