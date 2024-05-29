package com.example.deliboard.game

import retrofit2.http.GET
import retrofit2.http.Query

interface GameApi {

    @GET("stock/games?")
    suspend fun getGameList(
        @Query("storeId") storeId: String,
    ) : List<GameClass>
}