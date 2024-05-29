package com.example.deliboard.firebase


import android.app.NotificationManager
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.util.Log
import com.example.deliboard.BaseApi
import com.example.deliboard.detail.ThemeOnApi
import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class NotificationActionReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        if (intent.action == "SEND_THEME") {
            val client = BaseApi.getInstance().create(ThemeOnApi::class.java)

            val sharedPreferences = context.getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
            val roomLogId = sharedPreferences.getString("roomLogId", "")
            val requestBody = mapOf(
                "roomLogId" to roomLogId
            )

            client.themeRequest(requestBody).enqueue(object : Callback<SuccessResponse> {
                override fun onResponse(call : Call<SuccessResponse>, response: Response<SuccessResponse>) {
                    if (response.isSuccessful) {
                        val responseBody = response.body()
                        if (responseBody != null && responseBody.success) {
//                            Log.d("Theme", "Theme change success")
                        } else {
                            Log.d("Theme", "Theme change failed : $responseBody")
                        }
                    } else {
                        Log.d("Theme", "request failed : $response")
                    }
                }

                override fun onFailure(call: Call<SuccessResponse>, t: Throwable) {
                    Log.e("Theme", "$t")
                }
            })

            // 알림 매니저를 사용하여 알림 종료
            val notificationManager = context.getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            notificationManager.cancel(0)
        }
    }
}