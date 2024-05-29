package com.example.deliboard.login

import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST

interface LoginApi {
    @POST("room/login")
    fun sendStoreInfo(@Body requestBody: Map<String, String?>): Call<SuccessResponse>
}