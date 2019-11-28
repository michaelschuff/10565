package org.firstinspires.ftc.teamcode.blobdetect;

import java.util.ArrayList;

public class BlobDetector {
    int[][] image;
    int height,width;
    ArrayList<Blob> blobs;

    BlobDetector(int[][] image, int height, int width){
        this.height = height;
        this.width = width;
        this.image = new int[height][width];
        for(int i = 0; i < height; i++){
            for(int j = 0; j < width; j++){
                this.image[i][j] = image[i][j];
            }
        }
    }

    public void findBlobs(){
        int blobNum = 0;
        for(int i = 0; i < height; i++){
            Boolean prevBlob = false;
            for(int j = 0; j < width - 1; j++){
                if(image[i][j] == image[i][j+1] && !prevBlob){
                    blobs.add(new Blob(blobNum, i, j, 2,1, image[i][j]));
                    blobNum++;
                    prevBlob = true;
                }else if(image[i][j] == image[i][j+1]){
                    blobs.get(blobNum).addpixel();
                }else{
                    prevBlob = false;
                }
            }
        }
        for(int i = 0; i < blobs.size(); i++){
            for(int j = i+1; j < blobs.size(); j++){
                if(blobs.get(i).getY() == blobs.get(j).getY()-1){
                    if(((blobs.get(i).getX() < blobs.get(j).getX()) && (blobs.get(j).getX() < blobs.get(i).getX() + blobs.get(i).getWidth())) ||
                            ((blobs.get(j).getX() < blobs.get(i).getX() + blobs.get(i).getWidth()) && (blobs.get(i).getX() + blobs.get(i).getWidth() < blobs.get(j).getX() + blobs.get(j).getWidth()))){
                        blobs.get(i).concattenate(blobs.get(j));
                        blobs.remove(j);
                    }
                }
            }
        }
        for(int i = 0; i < blobs.size(); i++){
            if(blobs.get(i).getWidth() < 5 && blobs.get(i).getHeight() < 5){
                blobs.remove(i);
            }
        }
    }
}
