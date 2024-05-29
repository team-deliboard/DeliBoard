package com.example.deliboard.firebase

import android.content.Context
import androidx.work.OneTimeWorkRequest
import androidx.work.WorkManager
import java.util.concurrent.TimeUnit

fun scheduleNotificationAction(context: Context) {
    val workManager = WorkManager.getInstance(context)

    // 10초 후에 실행될 작업을 생성합니다.
    val notificationActionWorkRequest =
        OneTimeWorkRequest.Builder(NotificationActionWorker::class.java)
            .setInitialDelay(10, TimeUnit.SECONDS) // 10초 지연
            .build()

    // 작업을 WorkManager 추가하여 예약합니다.
    workManager.enqueue(notificationActionWorkRequest)
}