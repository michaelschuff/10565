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
        for(int i = 450; i < 550; i++){
            for(int j = 400; j < 550; j++){
                stoneOneAvg += (inArray[j + i*width] >> 16) & 0xFF;
            }
        }
        stoneOneAvg /= 15000;
        for(int i = 450; i < 550; i++){
            for(int j = 750; j < 900; j++){
                stoneTwoAvg += (inArray[j + i*width] >> 16) & 0xFF;
            }
        }
        stoneTwoAvg /= 15000;
        for(int i = 450; i < 550; i++){
            for(int j = 1100; j < 1250; j++){
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
