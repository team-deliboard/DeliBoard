package com.example.deliboard.room

import retrofit2.http.GET
import retrofit2.http.Query

interface HistoryApi {

    @GET("menu-order/room?")
    suspend fun getHistoryList(
        @Query("roomLogId") roomLogId: String,
    ) : List<HistoryClass>
}