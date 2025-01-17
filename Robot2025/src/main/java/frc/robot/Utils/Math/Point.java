package frc.robot.Utils.Math;

public class Point implements Comparable<Point>{
    public double x, y;
    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }
    
    @Override
    public int compareTo(Point o) {
        if(o.x == this.x){
            return 0;
        }
        else if(o.x > this.x){
            return -1;
        }
        return 1;
    }

    
}
