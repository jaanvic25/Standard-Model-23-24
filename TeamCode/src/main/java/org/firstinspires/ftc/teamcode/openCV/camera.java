package org.firstinspires.ftc.teamcode.openCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.AbstractMap.SimpleEntry;


public class camera extends OpenCvPipeline {
    Telemetry telemetry;
    SignalColor biggestArea = SignalColor.IDK;
    int zone = 0;

    public camera(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {



//        converts rgb to hsv
        //telemetry.addData("width", input.size().width);
        // width = 1920

        double width = input.size().width /3;
        double height = input.size().height;
        Mat rect1 = new Mat(input, new Rect(0, 0, (int)width, (int)height));
        Mat rect2 = new Mat(input, new Rect((int)width, 0, (int)width, (int)height));
        Mat rect3 = new Mat(input, new Rect(2*(int)width, 0, (int)width, (int)height));

//TODO: fix crop distribution (should not be 1/3, 2/3, 3/3)

        //add both red areas together - cant combine 174-180 and 0-7
        Scalar redLowerA = new Scalar(174, 112, 92);
        Scalar redUpperA = new Scalar(180, 255, 255);
        Scalar redLower = new Scalar(0, 112, 92);
        Scalar redUpper = new Scalar(6, 255, 255);
        Scalar purpleLower = new Scalar(135, 60, 76.5);
        Scalar purpleUpper = new Scalar(155, 204, 255);
        Scalar blueLower = new Scalar(95, 70, 92);
        Scalar blueUpper = new Scalar(124, 255, 240);
        /*double purpleArea = colorArea(input,purpleUpper,purpleLower,input);
        double redArea = colorArea(input,redUpper,redLower,input) + colorArea(input,redUpperA,redLowerA,input);
        double blueArea = colorArea(input,blueUpper,blueLower,input);

        if (purpleArea > redArea && purpleArea > blueArea) {
            biggestArea = SignalColor.PURPLE;
            area = purpleArea;
        } else if (redArea > purpleArea && redArea > blueArea) {
            biggestArea = SignalColor.RED;
            area = redArea;
        } else if (blueArea > purpleArea && blueArea > redArea) {
            biggestArea = SignalColor.BLUE;
            area = blueArea;
        } else {
            biggestArea = SignalColor.IDK;
        }*/
        // making list of areas [zone0, zone1, zone2] then finding max out of them
        double[] purpleAreas = {colorArea(rect1,purpleUpper,purpleLower,rect1),
                colorArea(rect2,purpleUpper,purpleLower,rect2), colorArea(rect3,purpleUpper,purpleLower,rect3)};
        SimpleEntry maxIndexP = maxIndex(purpleAreas);
        double[] redAreas = {colorArea(rect1,redUpper,redLower,rect1) + colorArea(rect1,redUpperA,redLowerA,rect1),
                colorArea(rect2,redUpper,redLower,rect2) + colorArea(rect2,redUpperA,redLowerA,rect2),
                colorArea(rect3,redUpper,redLower,rect3) + colorArea(rect3,redUpperA,redLowerA,rect3)};
        SimpleEntry maxIndexR = maxIndex(redAreas);
        double[] blueAreas = {colorArea(rect1,blueUpper,blueLower,rect1),
                colorArea(rect2,blueUpper,blueLower,rect2), colorArea(rect3,blueUpper,blueLower,rect3)};
        SimpleEntry maxIndexB = maxIndex(blueAreas);
        zone = 0; //as default

        final int AREA_LIMIT = 2000;
        double[] areas = {(double)maxIndexP.getKey(), (double)maxIndexR.getKey(), (double)maxIndexB.getKey()};
        double maxArea = Arrays.stream(areas).max().getAsDouble();
        if(maxArea == (double)maxIndexP.getKey()){
            biggestArea = SignalColor.PURPLE;
            zone = (int)maxIndexP.getValue();
        } else if(maxArea == (double)maxIndexR.getKey()){
            biggestArea = SignalColor.RED;
            zone = (int)maxIndexR.getValue();
        } else {
            biggestArea = SignalColor.BLUE;
            zone = (int)maxIndexB.getValue();
        }

        if (maxArea < AREA_LIMIT){
            biggestArea = SignalColor.IDK;
        }

        telemetry.addData("Biggest Area hi ", biggestArea);
        telemetry.addData("Zone ", zone);
        telemetry.update();


        return input;

    }

    public double colorArea(Mat mat, Scalar upper, Scalar lower, Mat input){
        Mat workingMat = new Mat();

        Imgproc.cvtColor(mat, workingMat, Imgproc.COLOR_RGB2HSV);

        //making pixels in the mat black and white based off a specific range - binary map of whats the color and whats not
        Core.inRange(workingMat, lower, upper, workingMat);

        //expand areas that are white - minimize the lone pixels, maximize the blobs (gets rid of some error)
        Imgproc.morphologyEx(workingMat,workingMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));

        //find contours

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(workingMat, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_TC89_KCOS);

        //biggest blob
        int index = 0;
        for (int i = 0; i < (contours.size()); i++) {
            MatOfPoint newContour = contours.get(i);
            Rect bound = Imgproc.boundingRect(newContour);

            //of a good blob size
            if(bound.width > 10 && bound.height >10) {
                if (Imgproc.contourArea(newContour) > Imgproc.contourArea(contours.get(index))) {
                    index = i;
                }
            }
        }

        double width = input.size().width /3;
        double height = input.size().height;

        Imgproc.rectangle(input, new Point(0, 0), new Point((int)width, (int)height), new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, new Point((int)width, 0), new Point((int)width, (int)height), new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, new Point(2*(int)width, 0), new Point((int)width, (int)height), new Scalar(0, 255, 0), 2);



        //done with this mat!!!
        workingMat.release();

        try{
            Imgproc.drawContours(input, contours, index, new Scalar(0,0,0));
            return Imgproc.contourArea(contours.get(index));
        } catch(Exception e){
            return 0;
        }


    }

    public SignalColor getBiggestArea() {
        return biggestArea;
    }

    public int getZone() {
        return zone;
    }

    public SimpleEntry<Double, Integer> maxIndex(double[] arr){
        double max = 0;
        int index = 0;
        for(int i = 0; i < arr.length; i++){
            if(arr[i]>max){
                max = arr[i];
                index = i;
            }
        }
        return new SimpleEntry(max, index);
    }
}
