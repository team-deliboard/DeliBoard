package com.example.deliboard.detail

import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.PUT
import retrofit2.http.Query

interface ThemeToggleApi {
    @GET("room/theme")
    fun themeCheck(
        @Query("roomLogId") roomLogId: String,
    ): Call<Boolean>

    @PUT("room/theme")
    fun sendThemeToggle(@Body requestBody: Map<String, String?>): Call<SuccessResponse>
}