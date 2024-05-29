package com.example.deliboard.login

import com.example.deliboard.room.RoomStartResponse
import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST
import retrofit2.http.PUT

interface StartApi {
    @POST("room")
    fun sendRoomInfo(@Body requestBody: Map<String, String?>): Call<RoomStartResponse>

    @PUT("room")
    fun deleteRoomInfo(@Body requestBody: Map<String, String?>): Call<SuccessResponse>
}