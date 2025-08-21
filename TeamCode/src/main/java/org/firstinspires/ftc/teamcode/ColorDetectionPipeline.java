package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

public class ColorDetectionPipeline extends OpenCvPipeline {

    private volatile double redPercentage = 0.0;
    private volatile double greenPercentage = 0.0;
    private volatile double bluePercentage = 0.0;
    private volatile String dominantColor = "None";

    private int totalPixels = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Calculate total pixels in the frame
        totalPixels = input.rows() * input.cols();

        // Convert image to HSV
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Create masks for each color
        Mat redMask1 = new Mat();
        Mat redMask2 = new Mat();
        Mat redMask = new Mat();
        Mat greenMask = new Mat();
        Mat blueMask = new Mat();

        // Red has two ranges in HSV (0-10 and 170-180)
        Core.inRange(hsv, new Scalar(0, 50, 50), new Scalar(10, 255, 255), redMask1);
        Core.inRange(hsv, new Scalar(170, 50, 50), new Scalar(180, 255, 255), redMask2);
        Core.add(redMask1, redMask2, redMask);

        // Green range
        Core.inRange(hsv, new Scalar(40, 50, 50), new Scalar(80, 255, 255), greenMask);

        // Blue range
        Core.inRange(hsv, new Scalar(100, 50, 50), new Scalar(130, 255, 255), blueMask);

        // Apply morphological operations to reduce noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(greenMask, greenMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);

        // Count pixels for each color
        int redCount = Core.countNonZero(redMask);
        int greenCount = Core.countNonZero(greenMask);
        int blueCount = Core.countNonZero(blueMask);

        // Calculate percentages
        redPercentage = (redCount * 100.0) / totalPixels;
        greenPercentage = (greenCount * 100.0) / totalPixels;
        bluePercentage = (blueCount * 100.0) / totalPixels;

        // Determine dominant color
        if (redPercentage > greenPercentage && redPercentage > bluePercentage && redPercentage > 0.5) {
            dominantColor = "Red";
        } else if (greenPercentage > redPercentage && greenPercentage > bluePercentage && greenPercentage > 0.5) {
            dominantColor = "Green";
        } else if (bluePercentage > redPercentage && bluePercentage > greenPercentage && bluePercentage > 0.5) {
            dominantColor = "Blue";
        } else {
            dominantColor = "None";
        }

        // Draw color percentage bars on the image
        drawPercentageBars(input);

        // Add text information
        addColorInfo(input);

        // Cleanup
        redMask1.release();
        redMask2.release();
        redMask.release();
        greenMask.release();
        blueMask.release();
        hsv.release();
        kernel.release();

        return input;
    }

    private void drawPercentageBars(Mat input) {
        int barWidth = 200;
        int barHeight = 20;
        int startX = 20;
        int startY = 60;
        int spacing = 30;

        // Red bar
        Imgproc.rectangle(input,
                new Point(startX, startY),
                new Point(startX + barWidth, startY + barHeight),
                new Scalar(100, 100, 100), -1); // Gray background

        int redBarWidth = (int)(barWidth * redPercentage / 100.0);
        if (redBarWidth > 0) {
            Imgproc.rectangle(input,
                    new Point(startX, startY),
                    new Point(startX + redBarWidth, startY + barHeight),
                    new Scalar(0, 0, 255), -1); // Red fill
        }

        // Green bar
        startY += spacing;
        Imgproc.rectangle(input,
                new Point(startX, startY),
                new Point(startX + barWidth, startY + barHeight),
                new Scalar(100, 100, 100), -1); // Gray background

        int greenBarWidth = (int)(barWidth * greenPercentage / 100.0);
        if (greenBarWidth > 0) {
            Imgproc.rectangle(input,
                    new Point(startX, startY),
                    new Point(startX + greenBarWidth, startY + barHeight),
                    new Scalar(0, 255, 0), -1); // Green fill
        }

        // Blue bar
        startY += spacing;
        Imgproc.rectangle(input,
                new Point(startX, startY),
                new Point(startX + barWidth, startY + barHeight),
                new Scalar(100, 100, 100), -1); // Gray background

        int blueBarWidth = (int)(barWidth * bluePercentage / 100.0);
        if (blueBarWidth > 0) {
            Imgproc.rectangle(input,
                    new Point(startX, startY),
                    new Point(startX + blueBarWidth, startY + barHeight),
                    new Scalar(255, 0, 0), -1); // Blue fill
        }
    }

    private void addColorInfo(Mat input) {
        // Add percentage text next to bars
        int textX = 230;
        int startY = 75;
        int spacing = 30;

        String redText = String.format("Red: %.2f%%", redPercentage);
        Imgproc.putText(input, redText, new Point(textX, startY),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);

        startY += spacing;
        String greenText = String.format("Green: %.2f%%", greenPercentage);
        Imgproc.putText(input, greenText, new Point(textX, startY),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);

        startY += spacing;
        String blueText = String.format("Blue: %.2f%%", bluePercentage);
        Imgproc.putText(input, blueText, new Point(textX, startY),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);

        // Add dominant color at the top
        String dominantText = "Dominant: " + dominantColor;
        Imgproc.putText(input, dominantText, new Point(20, 30),
                Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 255), 2);
    }

    // Getter methods
    public double getRedPercentage() {
        return redPercentage;
    }

    public double getGreenPercentage() {
        return greenPercentage;
    }

    public double getBluePercentage() {
        return bluePercentage;
    }

    public String getDominantColor() {
        return dominantColor;
    }

    public int getTotalPixels() {
        return totalPixels;
    }
}