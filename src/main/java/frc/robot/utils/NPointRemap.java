package frc.robot.utils;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Scanner;

import org.opencv.core.Point;

public class NPointRemap {
    private ArrayList<Point> points;
    private boolean isSorted;
    private File file;

    private class PointComparator implements Comparator<Point> {
        @Override
        public int compare(Point p1, Point p2) {
            return (int) Math.signum(p2.x - p1.x); // Descending Order
        }
    }

    public NPointRemap(String filePath) {
        points = new ArrayList<Point>();
        isSorted = false;
        file = new File(filePath);
        readFromFile();
    }

    public void compile() {
        Collections.sort(points, new PointComparator());
        isSorted = true;

        try {
            FileWriter fw = new FileWriter(file);
            file.delete();
            for(int i = 0; i < points.size(); i++) {
                fw.write(points.get(i).x + " " + points.get(i).y + "\n");
            }
            fw.close();
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    public NPointRemap addPoint(double x, double y) {
        isSorted = false;
        points.add(new Point(x, y));
        return this;
    }

    public double calcY(double x) {
        if (points.size() >= 2) {
            if (!isSorted) {
                compile();
            }
            for (int i = 0; i < points.size() - 1; i++) {
                if (x >= points.get(i + 1).x) {
                    return MathUtils.remap(points.get(i + 1).x, points.get(i).x, x, points.get(i + 1).y,
                            points.get(i).y);
                }
            }
        }
        return 0;
    }

    public void readFromFile() {
        if(file.exists()) {
            try {
                Scanner sc = new Scanner(file);
                while(true) {
                    if(sc.hasNext()) {
                        double x = sc.nextDouble();
                        double y = sc.nextDouble();
                        addPoint(x, y);
                    } else {
                        break;
                    }
                }
                isSorted = true;
                sc.close();
                return;
            } catch(Exception e) {
                e.printStackTrace();
            }
        }
    }
}