package frc.robot.Utils.Math;

import java.util.Arrays;


public class LinearInterpolation {
    private Point[] m_data;

    public LinearInterpolation(Point... data) {
        m_data = data;
        Arrays.sort(m_data);
    }

    public double interpolate(double x) {
        if(m_data.length == 0){
            System.out.println("Cant");
            return 0;
        }
        if (x > m_data[m_data.length - 1].x) {
            return m_data[m_data.length - 1].y;
        }
        if (x < m_data[0].x) {
            return m_data[0].y;
        }
        int i = 0;
        while (x > m_data[i].x) {
            i++;
        }
        double m = (m_data[i].y - m_data[i-1].y) / (m_data[i].x - m_data[i-1].x);//m is the slope
        double b = m_data[i].y - m_data[i].x * m;
        return m*x+b;
    }
}