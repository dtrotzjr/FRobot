package com.apppoetry.FMote;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.drawable.ShapeDrawable;
import android.graphics.drawable.shapes.ArcShape;
import android.graphics.drawable.shapes.OvalShape;
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
        int x = 10;
        int y = 10;
        int width = 300;
        int height = 50;

//        mDrawable = new ShapeDrawable(new OvalShape());
//        mDrawable.getPaint().setColor(0xff74AC23);
//        mDrawable.setBounds(x, y, x + width, y + height);

        mAngleValuePairs = null;
        mAngleValuePairLength = 0;
        mSonarResultMessage = "";
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
                        mAngleValuePairs[pairIndex++] = new AngleValuePair(pairTokenizer.nextToken(), pairTokenizer.nextToken());
                    }
                }
            }
        }
        mAngleValuePairLength = pairIndex;
    }

    protected void onDraw(Canvas canvas) {
        parseResultMessage();

        for (int pairIndex = 0; pairIndex < mAngleValuePairLength; pairIndex++) {
            AngleValuePair pair = mAngleValuePairs[pairIndex];
            ShapeDrawable drawable = new ShapeDrawable(new ArcShape(pair.getAngle() - 7, 14));
            drawable.setAlpha(128);
            drawable.getPaint().setColor(0x7F74AC23);
            drawable.setBounds(10, 10, getMeasuredWidth() - 20, getMeasuredHeight() - 20);
            drawable.draw(canvas);
        }


    }
}