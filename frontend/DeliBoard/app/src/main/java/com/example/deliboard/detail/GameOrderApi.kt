package com.example.deliboard.detail

import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.http.GET
import retrofit2.http.Query

interface GameOrderApi {
    @GET("game-order?")
    fun orderGame(
        @Query("gameId") gameId: String,
        @Query("roomLogId") roomLogId: String
    ): Call<SuccessResponse>
}