package com.example.eda.continuousshooting;

import android.content.Context;
import android.os.Handler;
import android.widget.Toast;

import java.util.TimerTask;

public class CameraTrigger extends TimerTask {

    private Handler mHandler;
    private Context mContext;

    public CameraTrigger(Context context,Handler handler) {
        mHandler = handler;
        this.mContext = context;
    }

    @Override
    public void run() {
        mHandler.post(new Runnable() {
            @Override
            public void run() {
                //Toast.makeText(mContext,"CameraTrigger",Toast.LENGTH_LONG).show();
                ((MainActivity)mContext).statejudge();
            }
        });
    }
}
