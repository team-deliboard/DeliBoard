package com.example.deliboard.firebase

import android.content.Context
import android.content.Intent
import androidx.work.Worker
import androidx.work.WorkerParameters
import android.util.Log

class NotificationActionWorker(appContext: Context, workerParams: WorkerParameters) :
    Worker(appContext, workerParams) {

    override fun doWork(): Result {
        val intent = Intent(applicationContext, NotificationActionReceiver::class.java)
        intent.action = "SEND_THEME" // 여기에 액션을 지정하세요
        applicationContext.sendBroadcast(intent)

        Log.d("NotificationActionWorker", "Notification action executed!")
        return Result.success()
    }
}