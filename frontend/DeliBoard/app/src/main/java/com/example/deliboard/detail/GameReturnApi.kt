package com.example.deliboard.detail

import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.http.GET
import retrofit2.http.Query

interface GameReturnApi {
    @GET("game-order/return?")
    fun returnGame(
        @Query("roomLogId") roomLogId: String
    ): Call<SuccessResponse>
}