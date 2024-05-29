package com.example.deliboard.manager

data class ManageClass(
    val id: Int,
    val menuName: String,
    val quantity: Int,
    val createdDttm: String,
    val roomLogId: String,
    val isDelivered: Boolean
)
