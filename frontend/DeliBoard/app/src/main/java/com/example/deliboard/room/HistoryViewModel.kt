package com.example.deliboard.room

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.deliboard.BaseApi
import kotlinx.coroutines.launch

class HistoryViewModel(private val roomLogId: String) : ViewModel() {
    private val HisrotyNetwork = BaseApi.getInstance().create(HistoryApi::class.java)

    private val _historyList = MutableLiveData<List<HistoryClass>>()
    val historyList: LiveData<List<HistoryClass>> = _historyList

    fun getHistoryList() {
        viewModelScope.launch {
            try {
                val result = HisrotyNetwork.getHistoryList(roomLogId)
                println("result: $result")
                _historyList.value = result
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}