package com.example.deliboard.game

data class GameClass(
    val id: Int,
    val title: String,
    val description: String,
    val thumbnailUrl: String, //이미지 url
    val stock: Int, // 재고
    val minPlayer: Int,
    val maxPlayer: Int,
    val minPlaytime: Int,
    val maxPlaytime: Int,
    val difficulty: Double, // 10이하 소수점 3자리
    val theme: String, // 테마
)