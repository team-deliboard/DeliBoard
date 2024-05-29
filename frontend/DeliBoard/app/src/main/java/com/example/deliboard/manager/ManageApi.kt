package com.example.deliboard.manager

import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.PUT
import retrofit2.http.Query

interface ManageApi {
    @GET("menu-order/orderList?")
    suspend fun getOrderList(
        @Query("storeId") storeId: String,
    ) : List<ManageClass>

    @GET("turtle/call?")
    fun callTurtle(
        @Query("roomLogId") roomLogId: String,
    ) : Call<SuccessResponse>

    @PUT("menu-order")
    fun sendMenuDeliverded(@Body requestBody: Map<String, Int?>): Call<SuccessResponse>
}