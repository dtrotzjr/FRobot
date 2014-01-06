/**
 *  @version 1.1 (28.01.2013)
 *  http://english.cxem.net/arduino/arduino5.php
 *  @author Koltykov A.V. (�������� �.�.)
 *
 */

package com.apppoetry.FMote;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.Method;
import java.util.UUID;
import java.util.logging.Level;
import java.util.logging.Logger;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

public class MainActivity extends Activity {
    private static final String TAG = "FMote";

    Button mBtnAuto;
    ImageButton mBtnThrottle;
    ImageButton mBtnSteering;

    TextView mLblClearance;
    TextView mLblAuto;
    TextView mLblMoving;

    boolean mThrottleActionEngaged;
    float mThrottleYRef;

    boolean mSteeringActionEngaged;
    float mSteeringXRef;

    boolean mAutoMode;

    SonarDrawableView mSonarView;

    Handler mSerialMessageHandler;

    final int RECIEVE_MESSAGE = 1;		// Status  for Handler
    private BluetoothAdapter mBtAdapter = null;
    private BluetoothSocket mBtSocket = null;
    private StringBuilder sb = new StringBuilder();

    private ConnectedThread mConnectedThread;

    // SPP UUID service
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    // MAC-Address of Bluetooth module (you must edit this line)
    private static String BT_MAC_ADDRESS = "20:13:12:02:16:26";

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        mBtnAuto = (Button) findViewById(R.id.btnAuto);
        mBtnThrottle = (ImageButton)findViewById(R.id.btnThrottle);
        mBtnSteering = (ImageButton) findViewById(R.id.btnSteering);
        mLblAuto = (TextView) findViewById(R.id.lblAuto);
        mLblClearance = (TextView) findViewById(R.id.lblClearance);
        mLblMoving = (TextView) findViewById(R.id.lblMoving);
        mSonarView = (SonarDrawableView)findViewById(R.id.viewSonar);

        mAutoMode = true;
        mThrottleActionEngaged = false;
        mThrottleYRef = 0.0f;
        mSteeringActionEngaged = false;
        mSteeringXRef = 0.0f;

        mSerialMessageHandler = new Handler() {
            public void handleMessage(android.os.Message msg) {
                switch (msg.what) {
                    case RECIEVE_MESSAGE:
                        String message = (String) msg.obj;
                        int argIndex = message.indexOf(':');
                        String arg = message.substring(argIndex + 1);
                        if (message.startsWith("FS+AUTO:")) {
                            mAutoMode = arg.equals("1");
                            mBtnAuto.setText(mAutoMode ? "OVERRIDE" : "AUTO");
                        } else if(message.startsWith("FS+MOV:")) {
                            String moving = arg.equals("1") ? "Moving" : "Stopped";
                            mLblMoving.setText(moving);
                        } else if(message.startsWith("FS+CLR:")) {
                            mLblClearance.setText(arg);
                        } else if(message.startsWith("FS+SONAR:")) {
                            mSonarView.setSonarResultMessage(arg);
                        }
                        break;
                }
            };
        };

        mBtAdapter = BluetoothAdapter.getDefaultAdapter();		// get Bluetooth adapter
        checkBTState();

        mBtnAuto.setOnClickListener(new OnClickListener() {
            public void onClick(View v) {
                if (mAutoMode)
                    mConnectedThread.write("FR+OVERRIDE");
                else
                    mConnectedThread.write("FR+AUTO");
            }
        });

        mBtnThrottle.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View view, MotionEvent motionEvent) {
                float y = motionEvent.getY();
                if ((motionEvent.getActionMasked() == MotionEvent.ACTION_DOWN) && !mThrottleActionEngaged)
                {
                    mThrottleActionEngaged = true;
                    mThrottleYRef = y;
                } else if ((motionEvent.getActionMasked() & MotionEvent.ACTION_UP) != 0)
                {
                    mThrottleActionEngaged = false;
                    mThrottleYRef = 0;
                    mConnectedThread.write("FR+DPAD:S:1\n");
                } else if ((motionEvent.getActionMasked() & MotionEvent.ACTION_MOVE) != 0 && mThrottleActionEngaged)
                {
                    float deltaY = y - mThrottleYRef;
                    if (deltaY < 10.0f) {
                        int magnitude = (-1.5f * deltaY) > 255.0 ? 255 : (int)(-1.5f *deltaY);
                        mConnectedThread.write("FR+DPAD:FWD:" + magnitude + "\n");
                    } else if (deltaY > 10.0f) {
                        int magnitude = (1.5f*deltaY) > 255.0 ? 255 : (int)(1.5f * deltaY);
                        mConnectedThread.write("FR+DPAD:REV:" + magnitude + "\n");
                    } else {
                        mConnectedThread.write("FR+DPAD:S:0\n");

                    }
                }
                return false;
            }
        });

        mBtnSteering.setOnTouchListener(new View.OnTouchListener() {
            public boolean onTouch(View view, MotionEvent motionEvent) {
                float x = motionEvent.getX();
                if ((motionEvent.getActionMasked() == MotionEvent.ACTION_DOWN) && !mSteeringActionEngaged)
                {
                    mSteeringActionEngaged= true;
                    mSteeringXRef = x;
                } else if ((motionEvent.getActionMasked() & MotionEvent.ACTION_UP) != 0)
                {
                    mSteeringActionEngaged = false;
                    mSteeringXRef = 0;
                    mConnectedThread.write("FR+DPAD:LT:0\n");
                } else if ((motionEvent.getActionMasked() & MotionEvent.ACTION_MOVE) != 0 && mSteeringActionEngaged)
                {
                    float deltaX = x - mSteeringXRef;
                    if (deltaX < 10.0f) {
                        int magnitude = (-0.15f * deltaX) > 25.0 ? 25 : (int)(-0.15f *deltaX);
                        mConnectedThread.write("FR+DPAD:LT:" + magnitude + "\n");
                    } else if (deltaX > 10.0f) {
                        int magnitude = (0.15f * deltaX) > 25.0 ? 25 : (int)(0.15f *deltaX);
                        mConnectedThread.write("FR+DPAD:RT:" + magnitude + "\n");
                    } else {
                        mConnectedThread.write("FR+DPAD:LT:0\n");

                    }
                }
                return false;
            }
        });
    }

    private BluetoothSocket createBluetoothSocket(BluetoothDevice device) throws IOException {
        if(Build.VERSION.SDK_INT >= 10){
            try {
                final Method  m = device.getClass().getMethod("createInsecureRfcommSocketToServiceRecord", new Class[] { UUID.class });
                return (BluetoothSocket) m.invoke(device, MY_UUID);
            } catch (Exception e) {
                Log.e(TAG, "Could not create Insecure RFComm Connection",e);
            }
        }
        return  device.createRfcommSocketToServiceRecord(MY_UUID);
    }

    @Override
    public void onResume() {
        super.onResume();

        Log.d(TAG, "...onResume - try connect...");

        // Set up a pointer to the remote node using it's BT_MAC_ADDRESS.
        BluetoothDevice device = mBtAdapter.getRemoteDevice(BT_MAC_ADDRESS);

        // Two things are needed to make a connection:
        //   A MAC BT_MAC_ADDRESS, which we got above.
        //   A Service ID or UUID.  In this case we are using the
        //     UUID for SPP.

        try {
            mBtSocket = createBluetoothSocket(device);
        } catch (IOException e) {
            errorExit("Fatal Error", "In onResume() and socket create failed: " + e.getMessage() + ".");
        }
    
    /*try {
      mBtSocket = device.createRfcommSocketToServiceRecord(MY_UUID);
    } catch (IOException e) {
      errorExit("Fatal Error", "In onResume() and socket create failed: " + e.getMessage() + ".");
    }*/

        // Discovery is resource intensive.  Make sure it isn't going on
        // when you attempt to connect and pass your message.
        mBtAdapter.cancelDiscovery();

        // Establish the connection.  This will block until it connects.
        Log.d(TAG, "...Connecting...");
        try {
            mBtSocket.connect();
            Log.d(TAG, "....Connection ok...");
        } catch (IOException e) {
            try {
                mBtSocket.close();
            } catch (IOException e2) {
                errorExit("Fatal Error", "In onResume() and unable to close socket during connection failure" + e2.getMessage() + ".");
            }
        }

        // Create a data stream so we can talk to server.
        Log.d(TAG, "...Create Socket...");

        mConnectedThread = new ConnectedThread(mBtSocket);
        mConnectedThread.start();
    }

    @Override
    public void onPause() {
        super.onPause();

        Log.d(TAG, "...In onPause()...");

        try     {
            mBtSocket.close();
        } catch (IOException e2) {
            errorExit("Fatal Error", "In onPause() and failed to close socket." + e2.getMessage() + ".");
        }
    }

    private void checkBTState() {
        // Check for Bluetooth support and then check to make sure it is turned on
        // Emulator doesn't support Bluetooth and will return null
        if(mBtAdapter ==null) {
            errorExit("Fatal Error", "Bluetooth not support");
        } else {
            if (mBtAdapter.isEnabled()) {
                Log.d(TAG, "...Bluetooth ON...");
            } else {
                //Prompt user to turn on Bluetooth
                Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableBtIntent, 1);
            }
        }
    }

    private void errorExit(String title, String message){
        Toast.makeText(getBaseContext(), title + " - " + message, Toast.LENGTH_LONG).show();
        finish();
    }

    private class ConnectedThread extends Thread {
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectedThread(BluetoothSocket socket) {
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            // Get the input and output streams, using temp objects because
            // member streams are final
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) { }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        public void run() {
            char[] buffer = new char[8192];  // buffer store for the stream
            int bytes; // bytes returned from read()

            // Keep listening to the InputStream until an exception occurs
            int bufferIndex = 0;
            while (true) {
                try {
                    // Read from the InputStream
                    int b = mmInStream.read();
                    if (b == '\n') {
                        String message = new String(buffer, 0, bufferIndex == 0 ? bufferIndex : bufferIndex - 1);
                        mSerialMessageHandler.obtainMessage(RECIEVE_MESSAGE, message).sendToTarget();
                        bufferIndex = 0;
                    } else if (b > 0) {
                        buffer[bufferIndex++] = (char)b;
                        if (bufferIndex > 255)
                            bufferIndex = 0;
                    }
                } catch (IOException e) {
                    break;
                }
            }
        }

        /* Call this from the main activity to send data to the remote device */
        public void write(String message) {
            Log.d(TAG, "...Data to send: " + message + "...");
            byte[] msgBuffer = message.getBytes();
            try {
                mmOutStream.write(msgBuffer);
            } catch (IOException e) {
                Log.d(TAG, "...Error data send: " + e.getMessage() + "...");
            }
        }
    }
}