package org.firstinspires.ftc.teamcode.Utilities;

public class Vector2 {
    public double x;

    public double y;
    public Vector2(){
        this.x = 0.0;
        this.y = 0.0;
    }
    public Vector2(double x,double y){
        this.x = x;
        this.y = y;
    }
    public Vector2(Vector2 vector){
        x = vector.x;
        y = vector.y;
    }
    public Vector2 add(Vector2 vector){
        return new Vector2(x + vector.x,y + vector.y);
    }
    public Vector2 sub(Vector2 vector){
        return new Vector2(x-vector.x, y-vector.y);
    }
    public Vector2 multiplyScalar(double scale){
        return new Vector2(x*scale, y*scale);
    }

    public double magnitude(){
        return Math.sqrt(x*x+y*y);
    }

    public Vector2 unit(){
        return multiplyScalar(1/magnitude());
    }

    public double dot(Vector2 vector){
        return x * vector.x + y * vector.y;
    }

    public Vector2 clone(){
        return new Vector2(this);
    }
}
