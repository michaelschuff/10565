package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.util.VuforiaLib_Skystone;
import android.graphics.Bitmap;


public class SkyStoneFinder {

    public static int detectSkystone(VuforiaLib_Skystone mvLib, boolean red){
        Bitmap bmIn = mvLib.getBitmap(4);
        int[] inArray = new int[bmIn.getWidth() * bmIn.getHeight()];
        bmIn.getPixels(inArray, 0, bmIn.getWidth(), 0, 0, bmIn.getWidth(), bmIn.getHeight());
        int width = bmIn.getWidth();
        int height = bmIn.getHeight();
        int stoneOneAvg = 0, stoneTwoAvg = 0, stoneThreeAvg = 0;
        int skyStone;
        if(!red) {
            for (int i = height * 410 / 900; i < height * 510 / 900; i++) {
                for (int j = width * 550 / 1600; j < width * 700 / 1600; j++) {
                    stoneOneAvg += (inArray[j + i * width] >> 16) & 0xFF;
                }
            }
            stoneOneAvg /= (height * 100 / 900) * (width * 150 / 1600);
            for (int i = height * 410 / 900; i < height * 510 / 900; i++) {
                for (int j = width * 850 / 1600; j < width * 1000 / 1600; j++) {
                    stoneTwoAvg += (inArray[j + i * width] >> 16) & 0xFF;
                }
            }
            stoneTwoAvg /= (height * 100 / 900) * (width * 150 / 1600);
            for (int i = height * 410 / 900; i < height * 510 / 900; i++) {
                for (int j = width * 1150 / 1600; j < width * 1300 / 1600; j++) {
                    stoneThreeAvg += (inArray[j + i * width] >> 16) & 0xFF;
                }
            }
            stoneThreeAvg /= (height * 100 / 900) * (width * 150 / 1600);
        }else{
            for (int i = height * 410 / 900; i < height * 510 / 900; i++) {
                for (int j = width * 400 / 1600; j < width * 550 / 1600; j++) {
                    stoneOneAvg += (inArray[j + i * width] >> 16) & 0xFF;
                }
            }
            stoneOneAvg /= (height * 100 / 900) * (width * 150 / 1600);
            for (int i = height * 410 / 900; i < height * 510 / 900; i++) {
                for (int j = width * 700 / 1600; j < width * 850 / 1600; j++) {
                    stoneTwoAvg += (inArray[j + i * width] >> 16) & 0xFF;
                }
            }
            stoneTwoAvg /= (height * 100 / 900) * (width * 150 / 1600);
            for (int i = height * 410 / 900; i < height * 510 / 900; i++) {
                for (int j = width * 1000 / 1600; j < width * 1150 / 1600; j++) {
                    stoneThreeAvg += (inArray[j + i * width] >> 16) & 0xFF;
                }
            }
            stoneThreeAvg /= (height * 100 / 900) * (width * 150 / 1600);
        }
        if(stoneOneAvg < stoneTwoAvg && stoneOneAvg < stoneThreeAvg){
            skyStone = 2;
        }else if(stoneTwoAvg < stoneOneAvg && stoneTwoAvg < stoneThreeAvg){
            skyStone = 1;
        }else{
            skyStone = 0;
        }
        return skyStone;
    }

}
