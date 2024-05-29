package com.example.deliboard.beverage

import com.example.deliboard.beverage.menus.MenuClass
import retrofit2.http.GET
import retrofit2.http.Query

interface BeverageApi {
    @GET("menu/list?")
    suspend fun getMenuList(
        @Query("storeId") storeId: String,
    ) : List<MenuClass>
}