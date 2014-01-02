package com.apppoetry.FMote;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.drawable.ShapeDrawable;
import android.graphics.drawable.shapes.ArcShape;
import android.graphics.drawable.shapes.OvalShape;
import android.graphics.drawable.shapes.RectShape;
import android.util.AttributeSet;
import android.view.View;

import java.lang.reflect.Array;
import java.util.StringTokenizer;

/**
 * Created by dctrotz on 1/1/14.
 */
public class SonarDrawableView extends View {
    private String mSonarResultMessage;

    private AngleValuePair mAngleValuePairs[];
    private int mAngleValuePairLength;
    private final int MAX_ANGLE_VALUE_PAIRS = 32;
    private int mMaxValue;
    private int mMinValue;

    private class AngleValuePair {
        private int mAngle;
        private int mValue;

        AngleValuePair(int angle, int value) {
            mAngle = angle;
            mValue = value;
        }

        AngleValuePair(String angleStr, String valueStr) {
            mAngle = (int)Math.round(Double.parseDouble(angleStr));
            mValue = (int)Math.round(Double.parseDouble(valueStr));
        }

        public int getAngle() { return  mAngle; }
        public int getValue() { return mValue; }
    }

    private void init() {
        mAngleValuePairs = null;
        mAngleValuePairLength = 0;
        mSonarResultMessage = "";
        mMaxValue = 0;
        mMinValue = 99999;
    }

    public SonarDrawableView(Context context) {
        super(context);
        init();
    }

    public SonarDrawableView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public void setSonarResultMessage(String sonarResultMessage) {
        if (sonarResultMessage != null) {
            synchronized (mSonarResultMessage) {
                mSonarResultMessage = sonarResultMessage;
            }
            invalidate();
        }
    }



    public void parseResultMessage() {
        String sonarResultMessage = null;
        synchronized (mSonarResultMessage) {
            sonarResultMessage = mSonarResultMessage;
        }

        int pairIndex = 0;
        mAngleValuePairs = new AngleValuePair[MAX_ANGLE_VALUE_PAIRS];
        int startIndex = sonarResultMessage.indexOf('{');
        int endIndex = sonarResultMessage.indexOf("}");
        if (startIndex >= 0 && endIndex > 0) {
            StringTokenizer setTokenizer = new StringTokenizer(sonarResultMessage.substring(startIndex + 1, endIndex - 1), "|");
            while (setTokenizer.hasMoreTokens() && pairIndex < MAX_ANGLE_VALUE_PAIRS) {
                String angleValuePair = setTokenizer.nextToken();
                int subStartIndex = angleValuePair.indexOf("(");
                int subEndIndex = angleValuePair.indexOf(")");
                if (subStartIndex >= 0 && subEndIndex > 0) {
                    StringTokenizer pairTokenizer = new StringTokenizer(angleValuePair.substring(subStartIndex + 1, subEndIndex - 1), ",");
                    if(pairTokenizer.countTokens() == 2) {
                        mAngleValuePairs[pairIndex] = new AngleValuePair(pairTokenizer.nextToken(), pairTokenizer.nextToken());
                        int curValue = mAngleValuePairs[pairIndex].getValue();
                        if (curValue > mMaxValue)
                            mMaxValue = curValue;
                        if (curValue < mMinValue)
                            mMinValue = curValue;
                        pairIndex++;
                    }
                }
            }
        }
        mAngleValuePairLength = pairIndex;
    }

    protected void drawTruck(Canvas canvas)
    {
        int margin = 15;
        int maxDrawArea = getMeasuredWidth() < getMeasuredHeight() ? getMeasuredWidth() : getMeasuredHeight();
        ShapeDrawable truckBody = new ShapeDrawable(new RectShape());
        int truckLength = (int)((maxDrawArea / 2.0) - margin);
        int truckWidth = (int)((maxDrawArea / 6.0));
        int top = (int)((maxDrawArea - truckWidth) / 2.0);
        truckBody.setBounds(margin, top, margin + truckLength, top + truckWidth);
        truckBody.getPaint().setARGB(196,196,16,32);
        truckBody.draw(canvas);

        int wheelLen = 40;
        int wheelWidth = 20;
        ShapeDrawable truckWheel1 = new ShapeDrawable(new RectShape());
        top -= wheelWidth + 5;
        int left = margin;
        truckWheel1.setBounds(left, top, left + wheelLen, top + wheelWidth);
        truckWheel1.draw(canvas);

        ShapeDrawable truckWheel2 = new ShapeDrawable(new RectShape());
        left = margin + truckLength - wheelLen;
        truckWheel2.setBounds(left, top, left + wheelLen, top + wheelWidth);
        truckWheel2.draw(canvas);

        ShapeDrawable truckWheel3 = new ShapeDrawable(new RectShape());
        top += wheelWidth + 5 + truckWidth + 5;
        left = margin;
        truckWheel3.setBounds(left, top, left + wheelLen, top + wheelWidth);
        truckWheel3.draw(canvas);

        ShapeDrawable truckWheel4 = new ShapeDrawable(new RectShape());
        left = margin + truckLength - wheelLen;
        truckWheel4.setBounds(left, top, left + wheelLen, top + wheelWidth);
        truckWheel4.draw(canvas);
    }
    protected void onDraw(Canvas canvas) {
        parseResultMessage();

        int maxArcRadius = getMeasuredWidth() < getMeasuredHeight() ? getMeasuredWidth() : getMeasuredHeight();
        float scaleFactor = ((float)maxArcRadius/(float)mMaxValue);


        for (int pairIndex = 0; pairIndex < mAngleValuePairLength; pairIndex++) {
            AngleValuePair pair = mAngleValuePairs[pairIndex];
            ShapeDrawable drawable = new ShapeDrawable(new ArcShape(-pair.getAngle() - 7, 14));
            int value = pair.getValue();
            // Set the alpha based on how close the value is to the max
            int alphaValue = (int)(196.0*((float)value/(float)mMaxValue));
            drawable.getPaint().setColor(0xFF74AC23);
            drawable.getPaint().setAlpha(alphaValue);

            // Set the radius based on how close the value is to the max value
            int arcRadius = (int)(value * scaleFactor);
            if (arcRadius % 2 != 0)
                arcRadius--; // make it a power of two so that the arcs line up.
            int margin = (int)((double)(maxArcRadius - arcRadius) / 2.0);
            drawable.setBounds(margin, margin, margin + arcRadius, margin + arcRadius);
            drawable.draw(canvas);
        }

        drawTruck(canvas);
    }
}