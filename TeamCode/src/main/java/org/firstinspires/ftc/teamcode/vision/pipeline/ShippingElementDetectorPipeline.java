/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision.pipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * With this pipeline, we demonstrate how to change which stage of
 * is rendered to the viewport when the viewport is tapped. This is
 * particularly useful during pipeline development. We also show how
 * to get data from the pipeline to your OpMode.
 */
public class ShippingElementDetectorPipeline extends OpenCvPipeline
{
    final Mat blurredMat = new Mat();
    final Mat hsvMat = new Mat();
    final Mat hueMat = new Mat();
    final Mat saturationMat = new Mat();
    final Mat valueMat = new Mat();
    final Mat maskMat = new Mat();
    final Mat thresholdMat = new Mat();
    final Mat contoursOnFrameMat = new Mat();
    final List<MatOfPoint> contoursList = new ArrayList<>();
    final List<MatOfPoint> filteredContoursList = new ArrayList<>();
    ShippingElementPosition shippingElementPosition = ShippingElementPosition.NOT_DETECTED;
    double frameWidthThreshold;

    enum Stage
    {
        RAW_IMAGE,
        HUE,
        SATURATION,
        VALUE,
        THRESHOLD,
        CONTOURS_OVERLAYED_ON_FRAME,
    }

    public enum ShippingElementPosition
    {
        NOT_DETECTED,
        LEFT,
        MIDDLE,
        RIGHT
    }

    private Stage stageToRenderToViewport = Stage.CONTOURS_OVERLAYED_ON_FRAME;
    private final Stage[] stages = Stage.values();

    @Override
    public void onViewportTapped()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentStageNum = stageToRenderToViewport.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageToRenderToViewport = stages[nextStageNum];
    }

    @Override
    public void init(Mat input)
    {
        frameWidthThreshold = input.width() / 3.0;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        contoursList.clear();

        /*
         * This pipeline finds the contours of yellow blobs such as the Gold Mineral
         * from the Rover Ruckus game.
         */
        int blurRadius = (int) (2.702702702702703 + 0.5);
        int kernelSize = 6 * blurRadius + 1;
        Imgproc.GaussianBlur(input, blurredMat, new Size(kernelSize, kernelSize), blurRadius);

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsvMat, hueMat, 0);
        Core.extractChannel(hsvMat, saturationMat, 1);
        Core.extractChannel(hsvMat, valueMat, 2);

        Core.inRange(hsvMat, new Scalar(120, 40, 55), new Scalar(150, 255, 255), maskMat);
        Core.bitwise_and(valueMat, maskMat, thresholdMat);

        input.copyTo(contoursOnFrameMat);

        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        filteredContoursList.clear();
        MatOfPoint largestContour = null;
        double largestContourSize = 0;
        for (MatOfPoint contour :
                contoursList) {
            double area = Imgproc.contourArea(contour);
            if (area >= 1000)
            {
                if (area > largestContourSize)
                {
                    largestContour = contour;
                    largestContourSize = area;
                }
                Rect boundingRect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(contoursOnFrameMat, boundingRect, new Scalar(0, 0, 255), 2);
                filteredContoursList.add(contour);
            }
        }

        Imgproc.drawContours(contoursOnFrameMat, filteredContoursList, -1, new Scalar(0, 255, 0), 3);

        if (largestContour != null)
        {
            Rect boundingRect = Imgproc.boundingRect(largestContour);
            Imgproc.rectangle(contoursOnFrameMat, boundingRect, new Scalar(255, 0, 0), 2);
            List<MatOfPoint> largestContourList = new ArrayList<>();
            largestContourList.add(largestContour);
            Imgproc.drawContours(contoursOnFrameMat, largestContourList, 0, new Scalar(255, 0, 0), 3);
            Moments M = Imgproc.moments(largestContour);
            int cx = (int) (M.m10/M.m00);
            int cy = (int) (M.m01/M.m00);
            Imgproc.circle(contoursOnFrameMat, new Point(cx, cy), 5, new Scalar(255, 0, 0), -1);

            if (cx < frameWidthThreshold)
            {
                shippingElementPosition = ShippingElementPosition.LEFT;
            }
            else if (cx < frameWidthThreshold * 2)
            {
                shippingElementPosition = ShippingElementPosition.MIDDLE;
            }
            else
            {
                shippingElementPosition = ShippingElementPosition.RIGHT;
            }
        }
        else
        {
            shippingElementPosition = ShippingElementPosition.NOT_DETECTED;
        }

        switch (stageToRenderToViewport)
        {
            case HUE:
            {
                return hueMat;
            }

            case SATURATION:
            {
                return saturationMat;
            }

            case VALUE:
            {
                return valueMat;
            }

            case THRESHOLD:
            {
                return thresholdMat;
            }

            case CONTOURS_OVERLAYED_ON_FRAME:
            {
                return contoursOnFrameMat;
            }

            default:
            {
                return input;
            }
        }
    }

    public ShippingElementPosition getShippingElementPosition()
    {
        return shippingElementPosition;
    }
}