package seamcarving;

import astar.AStarGraph;
import astar.AStarSolver;
import astar.WeightedEdge;
import edu.princeton.cs.algs4.Picture;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

//Should be done
public class AStarSeamCarver implements SeamCarver {
    private Picture picture;

    public AStarSeamCarver(Picture picture) {
        if (picture == null) {
            throw new NullPointerException("Picture cannot be null.");
        }
        this.picture = new Picture(picture);
    }

    public Picture picture() {
        return new Picture(picture);
    }

    public void setPicture(Picture picture) {
        this.picture = picture;
    }

    public int width() {
        return picture.width();
    }

    public int height() {
        return picture.height();
    }

    public Color get(int x, int y) {
        return picture.get(x, y);
    }

    public double energy(int x, int y) {
        //Get X gradient
        Color left;
        if (x - 1 < 0) {
            left = get(picture.width() - 1, y);
        } else {
            left = get(x - 1, y);
        }
        Color right;
        if (x + 1 >= picture.width()) {
            right = get(0, y);
        } else {
            right = get(x + 1, y);
        }
        int hRed = Math.abs(left.getRed() - right.getRed());
        int hGreen = Math.abs(left.getGreen() - right.getGreen());
        int hBlue = Math.abs(left.getBlue() - right.getBlue());
        int xGrad = hRed * hRed + hGreen * hGreen + hBlue * hBlue;

        //Get Y gradient
        Color top;
        if (y - 1 < 0) {
            top = get(x, picture.height() - 1);
        } else {
            top = get(x, y - 1);
        }
        Color bottom;
        if (y + 1 >= picture.height()) {
            bottom = get(x, 0);
        } else {
            bottom = get(x, y + 1);
        }
        int vRed = Math.abs(top.getRed() - bottom.getRed());
        int vGreen = Math.abs(top.getGreen() - bottom.getGreen());
        int vBlue = Math.abs(top.getBlue() - bottom.getBlue());
        int yGrad = vRed * vRed + vGreen * vGreen + vBlue * vBlue;

        return Math.sqrt(xGrad + yGrad);
    }

    public int[] findHorizontalSeam() {
        HorizontalPictureGraph hpg = new HorizontalPictureGraph();
        AStarSolver<Point> horizSolver = new AStarSolver<>(hpg, hpg.start, hpg.end, Integer.MAX_VALUE);
        List<Point> soln = horizSolver.solution();
        soln.remove(0);
        soln.remove(soln.size() - 1);
        int[] cleanedSoln = new int[soln.size()];
        for (int i = 0; i < soln.size(); i++) {
            cleanedSoln[i] = soln.get(i).y;
        }
        return cleanedSoln;
    }

    public int[] findVerticalSeam() {
        VerticalPictureGraph vpg = new VerticalPictureGraph();
        AStarSolver<Point> vertSolver = new AStarSolver<>(vpg, vpg.start, vpg.end, Integer.MAX_VALUE);
        List<Point> soln = vertSolver.solution();
        soln.remove(0);
        soln.remove(soln.size() - 1);
        int[] cleanedSoln = new int[soln.size()];
        for (int i = 0; i < soln.size(); i++) {
            cleanedSoln[i] = soln.get(i).x;
        }
        return cleanedSoln;
    }

    private class VerticalPictureGraph implements AStarGraph<Point> {
        private Point start = new Point(-1, -1);
        private Point end = new Point(picture.width(), picture.height());

        @Override
        public java.util.List<WeightedEdge<Point>> neighbors(Point p) {
            List<WeightedEdge<Point>> neighbors = new ArrayList<>();
            if (p.equals(start)) {
                for (int i = 0; i < picture.width(); i++) {
                    Point next = new Point(i, 0);
                    neighbors.add(new WeightedEdge<>(start, next, energy(next.x, next.y)));
                }
                return neighbors;
            }

            if (p.y != picture.height() - 1) {
                if (p.x != 0) {
                    Point left = new Point(p.x - 1, p.y + 1);
                    neighbors.add(new WeightedEdge<>(p, left, energy(left.x, left.y)));
                }
                if (p.x != picture.width() - 1) {
                    Point right = new Point(p.x + 1, p.y + 1);
                    neighbors.add(new WeightedEdge<>(p, right, energy(right.x, right.y)));
                }
                Point mid = new Point(p.x, p.y + 1);
                neighbors.add(new WeightedEdge<>(p, mid, energy(mid.x, mid.y)));
            } else {
                neighbors.add(new WeightedEdge<>(p, end, 0));
            }

            return neighbors;
        }

        @Override
        public double estimatedDistanceToGoal(Point s, Point goal) {
            return 0;
        }
    }

    private class HorizontalPictureGraph implements AStarGraph<Point> {
        private Point start = new Point(-1, -1);
        private Point end = new Point(picture.width(), picture.height());

        @Override
        public java.util.List<WeightedEdge<Point>> neighbors(Point p) {
            List<WeightedEdge<Point>> neighbors = new ArrayList<>();
            if (p.equals(start)) {
                for (int i = 0; i < picture.height(); i++) {
                    Point next = new Point(0, i);
                    neighbors.add(new WeightedEdge<>(start, next, energy(next.x, next.y)));
                }
                return neighbors;
            }

            if (p.x != picture.width() - 1) {
                if (p.y != 0) {
                    Point top = new Point(p.x + 1, p.y - 1);
                    neighbors.add(new WeightedEdge<>(p, top, energy(top.x, top.y)));
                }
                if (p.y != picture.height() - 1) {
                    Point bottom = new Point(p.x + 1, p.y + 1);
                    neighbors.add(new WeightedEdge<>(p, bottom, energy(bottom.x, bottom.y)));
                }
                Point mid = new Point(p.x + 1, p.y);
                neighbors.add(new WeightedEdge<>(p, mid, energy(mid.x, mid.y)));
            } else {
                neighbors.add(new WeightedEdge<>(p, end, 0));
            }

            return neighbors;
        }

        @Override
        public double estimatedDistanceToGoal(Point s, Point goal) {
            return 0;
        }
    }
}


