package com.example.deliboard.beverage.cart

import com.example.deliboard.SuccessResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.POST

interface CartApi {
    @POST("menu-order")
    fun MenuOrder(@Body requestBody: MenuOrderRequest): Call<SuccessResponse>
}