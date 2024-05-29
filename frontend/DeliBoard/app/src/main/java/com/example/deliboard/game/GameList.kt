package com.example.deliboard.game

import android.content.Context
import android.os.Bundle
import android.view.KeyEvent
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.EditorInfo
import androidx.fragment.app.viewModels
import androidx.lifecycle.lifecycleScope
import androidx.recyclerview.widget.GridLayoutManager
import com.example.deliboard.BaseApi
import com.example.deliboard.databinding.FragmentGameListBinding
import com.example.deliboard.detail.ThemeToggleApi
import kotlinx.coroutines.launch
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class GameList : Fragment() {

    private lateinit var binding: FragmentGameListBinding

    private val viewModel: GameViewModel by viewModels()

    private lateinit var gameAdapter: GameAdapter
    private lateinit var originalGameList: List<GameClass>

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        // Inflate the layout for this fragment
        binding = FragmentGameListBinding.inflate(layoutInflater)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        gameAdapter = GameAdapter(requireContext(), emptyList()) // 초기에 빈 리스트로 어댑터 생성
        binding.gameListRV.adapter = gameAdapter
        binding.gameListRV.layoutManager = GridLayoutManager(requireContext(), 2)

        viewModel.getCurrentGameList()

        val sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)

        val roomLogId = sharedPreferences.getString("roomLogId", "")
        toggleCheck(roomLogId!!)


        lifecycleScope.launch {
            viewModel.gameList.observe(viewLifecycleOwner) { gamelist ->
                gameAdapter.updateData(gamelist) // 데이터 업데이트
                originalGameList = gamelist
            }
        }

        binding.buttonSearch.setOnClickListener{
            performSearch()
        }

        // EditText에서 엔터 키를 눌렀을 때
        binding.editTextSearch.setOnEditorActionListener { _, actionId, event ->
            if (actionId == EditorInfo.IME_ACTION_SEARCH ||
                (event != null && event.keyCode == KeyEvent.KEYCODE_ENTER)) {
                performSearch()
                true
            } else {
                false
            }
        }
    }

    private fun performSearch() {
        val searchText = binding.editTextSearch.text.toString().trim()
        if (searchText.isNotEmpty()) {
            val filteredList = originalGameList.filter {game ->
                game.title.contains(searchText, ignoreCase = true)
            }
            gameAdapter.updateData(filteredList)
        } else {
            gameAdapter.updateData(originalGameList)
        }
    }

    private fun toggleCheck(roomLogId: String) {
        val client = BaseApi.getInstance().create(ThemeToggleApi::class.java)

        client.themeCheck(roomLogId).enqueue(object : Callback<Boolean> {
            override fun onResponse(call: Call<Boolean>, response: Response<Boolean>) {
//                println("토글 켜져있는지 여부 $response")
                if (response.isSuccessful) {
//                    println(response.body())
                    val toggleData = response.body()
                    if (toggleData != null) {
                        saveThemeOnOff(toggleData)
                    } else {
                        // 응답에서 토글 데이터가 없는 경우에 대한 처리
                    }
                } else {
                    // 서버 응답이 실패한 경우에 대한 처리
                }
            }
            override fun onFailure(call: Call<Boolean>, t: Throwable) {

            }
        })
    }

    private fun saveThemeOnOff(bool: Boolean) {
        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
//        println("bool saved : $bool")
        editor.putBoolean("themeOn", bool)
        editor.apply()
    }
}