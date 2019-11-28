package org.firstinspires.ftc.teamcode.blobdetect;

import static java.lang.Math.abs;

public class Blob {
    static int id;
    int x, y;
    int width, height;
    int color;

    public Blob(int id, int x, int y, int width, int height, int color){
        this.id = id;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.color = color;
    }

    public void set(int id, int x, int y, int width, int height, int color){
        this.id = id;
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
        this.color = color;
    }

    public void concattenate(Blob newBlob){
        this.height = abs((newBlob.y + newBlob.height) - this.y);
        this.width = abs((newBlob.x + newBlob.width)- this.x);
        this.x = this.x < newBlob.x ? this.x : newBlob.x;
        this.y = this.y < newBlob.y ? this.y : newBlob.y;
        newBlob.id = -1;


    }

    public void addpixel(){
        this.width++;
    }

    public static int getId() {
        return id;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public int getColor() {
        return color;
    }


}
