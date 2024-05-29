package com.example.deliboard.detail

import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.POST
import retrofit2.http.Query

interface ThemeOnApi{
    @POST("room/theme")
    fun themeRequest(@Body requestBody: Map<String, String?>): Call<SuccessResponse>
}