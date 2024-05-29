package com.example.deliboard.game

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.launch

class GameViewModel() : ViewModel() {

    private val gameNetwork = GameNetwork()

    private val _gameList = MutableLiveData<List<GameClass>>()
    val gameList: LiveData<List<GameClass>> = _gameList

    fun getCurrentGameList() {
        viewModelScope.launch {
            try {
                val result = gameNetwork.getGameList()
                _gameList.value = result
            } catch (e: Exception) {
                // 에러 처리
                println(e)
            }
        }
    }
}