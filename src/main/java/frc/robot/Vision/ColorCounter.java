package frc.robot.Vision;

import org.opencv.core.*;
import org.opencv.imgproc.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import java.util.LinkedList;
import java.util.Optional;

public class ColorCounter {
    /**
     * Defines threshold values to define a color
     */
    public class Color {
        public final String name;
        public final double hueMin;
        public final double hueMax;
        public final double satMin;
        public final double satMax;
        public final double lumMin;
        public final double lumMax;

        /**
         * Constructor
         * 
         * @param name   color name
         * @param hueMin minimum hue threshold
         * @param hueMax maximum hue threshold
         * @param satMin minimum saturation threshold
         * @param satMax maximum saturation threshold
         * @param lumMin minimum luminance threshold
         * @param lumMax maximum luminance threshold
         */
        public Color(String name, double hueMin, double hueMax, double satMin, double satMax, double lumMin,
                double lumMax) {
            this.name = name;
            this.hueMin = Math.min(Math.min(hueMin, 0.0), hueMax);
            this.hueMax = Math.max(Math.max(hueMax, 255.0), hueMin);
            this.satMin = Math.min(Math.min(satMin, 0.0), satMax);
            this.satMax = Math.max(Math.max(satMax, 255.0), satMin);
            this.lumMin = Math.min(Math.min(lumMin, 0.0), lumMax);
            this.lumMax = Math.max(Math.max(lumMax, 255.0), lumMin);
        }
    }

    private final UsbCamera camera;

    private final CvSink cvSink;

    private final Mat img = new Mat();

    private final LinkedList<ColorResult> results = new LinkedList<>();
    private Optional<Color> bestColor = Optional.empty();
    private int countThreshold;

    private final Thread thread = new Thread(this::process);
    private final Object resultLock = new Object();

    /**
     * Defines the result of a color detection
     */
    public class ColorResult {
        public static final Scalar black = new Scalar(0, 0, 0);

        public final Scalar lowerThreshold;
        public final Scalar upperThreshold;

        public final Color color;
        private final CvSource imgStream;
        private final Mat img = new Mat();
        private int count = 0;

        /**
         * Constructor
         * 
         * @param color Color the result is for
         */
        public ColorResult(Color color) {
            this.color = color;
            this.lowerThreshold = new Scalar(color.hueMin, color.satMin, color.lumMin);
            this.upperThreshold = new Scalar(color.hueMax, color.satMax, color.lumMax);
            this.imgStream = CameraServer.putVideo(color.name + " found", 640, 480);
            // TODO Allow adjustments of output resolution
        }

        /**
         * Set the pixel count for the color
         * 
         * @param count Pixel count of the color
         */
        public void setCount(int count) {
            this.count = count;
        }

        /**
         * Get the pixel count for the color
         * 
         * @return Pixel count of the color
         */
        public int getCount() {
            return count;
        }

        /**
         * Counts the amount of the color in the image and outputs a binary image of the
         * regions with that color
         * 
         * @param input image to process
         */
        public void countColor(Mat input) {
            // Get binary image of the color
            Imgproc.cvtColor(input, img, Imgproc.COLOR_BGR2HLS);
            Core.inRange(img, lowerThreshold, upperThreshold, img);

            // Count white pixels in binary image
            count = Core.countNonZero(img);

            // Output the image to the image stream
            imgStream.putFrame(img);
        }

        /**
         * Clears the result
         */
        public void clear() {
            count = 0;
            img.setTo(black);
            imgStream.putFrame(img);
        }
    }

    /**
     * Constructor. Starts the vision thread.
     * 
     * @param cameraConfig   Camera configuration
     * @param countThreshold minimum count for a best target result
     * @param colors         list of colors to detect
     */
    public ColorCounter(CameraConfig cameraConfig, int countThreshold, Color... colors) {
        // Initialize Camera
        camera = cameraConfig.cameraID.isPresent()
                ? CameraServer.startAutomaticCapture(cameraConfig.cameraID.get())
                : CameraServer.startAutomaticCapture();

        if (cameraConfig.width.isPresent() && cameraConfig.height.isPresent()) {
            camera.setResolution(cameraConfig.width.get(), cameraConfig.height.get());
        }

        // Initialize Cv Sink to capture images from the camera
        cvSink = CameraServer.getVideo(camera);

        // Set best count threshold
        this.countThreshold = countThreshold;

        // Initialize color result list
        for (var color : colors)
            results.add(new ColorResult(color));

        // Start thread
        thread.setDaemon(true);
        thread.start();
    }


    /**
     * Get color with the highest pixel count
     * 
     * @return Color with the highest pixel count. Optional.empty if process has not
     *         run, or no color passes the countThreshold
     */
    public Optional<Color> getBestColor() {
        synchronized (resultLock) {
            return bestColor;
        }
    }

    /**
     * Thread entry method
     */
    private void process() {
        // Process images from camera
        while (!Thread.interrupted()) { // TODO Improve stop conditions

            if (cvSink.grabFrame(img) != 0) {
                // Find all the colors in the image
                int maxCount = 0;
                Color maxColor = null;
                for (var result : results) {
                    result.countColor(img);
                    if (maxCount < result.getCount()) {
                        maxCount = result.getCount();
                        maxColor = result.color;
                    }
                }

                // Determine the best color
                synchronized (resultLock) {
                    bestColor = maxCount > countThreshold ? Optional.of(maxColor) : Optional.empty();
                }

            } else {

                for (var result : results)
                    result.clear();

                synchronized (resultLock) {
                    bestColor = Optional.empty();
                }
            }
        }
    }
}
