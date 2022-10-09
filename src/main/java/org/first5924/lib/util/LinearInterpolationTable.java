package org.first5924.lib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

public class LinearInterpolationTable {
    private final ArrayList<ArrayList<Double>> mCoordinates = new ArrayList<>();

    public LinearInterpolationTable() {}

    public LinearInterpolationTable(ArrayList<ArrayList<Double>> coordinates) {
        mCoordinates.addAll(coordinates);
        this.sort();
    }

    @Override
    public String toString() {
        return mCoordinates.toString();
    }

    public void add(double x, double y) {
        mCoordinates.add(new ArrayList<>(Arrays.asList(x, y)));
    }

    public void add(ArrayList<ArrayList<Double>> coordinates) {
        mCoordinates.addAll(coordinates);
    }

    public void sort() {
        mCoordinates.sort(Comparator.comparing(a -> a.get(0)));
    }

    public double interpolate(double key) {
        ArrayList<Double> lowEntry;
        ArrayList<Double> highEntry;
        for (int i = 0; i < mCoordinates.size(); i++) {
            if (key <= mCoordinates.get(i).get(0)) {
                if (i == 0) {
                    System.out.println("Equal to zero");
                    if (key < mCoordinates.get(i).get(0)) {
                        throw new IndexOutOfBoundsException("Number is smaller than the key of the first entry");
                    } else {
                        return mCoordinates.get(0).get(1);
                    }
                } else {
                    lowEntry = mCoordinates.get(i - 1);
                    highEntry = mCoordinates.get(i);
                    double xZero = lowEntry.get(0);
                    double yZero = lowEntry.get(1);
                    double xOne = highEntry.get(0);
                    double yOne = highEntry.get(1);
                    // Linear interpolation formula
                    return (yZero * (xOne - key) + yOne * (key - xZero)) / (xOne - xZero);
                }
            }
        }
        throw new IndexOutOfBoundsException("Number is larger than the key of the last entry");
    }
}
