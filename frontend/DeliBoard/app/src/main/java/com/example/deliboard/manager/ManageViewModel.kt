package com.example.deliboard.manager

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.deliboard.game.GameClass
import kotlinx.coroutines.launch

class ManageViewModel() : ViewModel() {

    private val manageNetwork = ManageNetwork()

    private val _orderList = MutableLiveData<List<ManageClass>>()
    val orderList: LiveData<List<ManageClass>> = _orderList

    fun getCurrentOrderList() {
        viewModelScope.launch {
            try {
                val result = manageNetwork.getOrderList()
//                println("뷰모델 들어옴 : $result")
                _orderList.value = result
            } catch (e: Exception) {
                // 에러 처리
                println("뷰모델 에러남 $e")
            }
        }
    }
}