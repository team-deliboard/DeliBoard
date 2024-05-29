package com.example.deliboard.detail

import android.content.Context
import android.content.SharedPreferences
import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ImageView
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.widget.SwitchCompat
import androidx.lifecycle.lifecycleScope
import com.bumptech.glide.Glide
import com.bumptech.glide.request.target.DrawableImageViewTarget
import com.example.deliboard.BaseApi
import com.example.deliboard.R
import com.example.deliboard.databinding.FragmentGameDetailBinding
import com.example.deliboard.SuccessResponse
import kotlinx.coroutines.launch
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class GameDetail : Fragment() {

    private lateinit var binding: FragmentGameDetailBinding
    private lateinit var sharedPreferences: SharedPreferences
    private lateinit var loadingImage: ImageView

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding = FragmentGameDetailBinding.inflate(inflater, container, false)
        return binding.root
    }


    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val backButton = view.findViewById<ImageView>(R.id.detailBack)

        loadingImage = view.findViewById<ImageView>(R.id.loadingImage)
        loadingImage.visibility = View.GONE

        Glide.with(this)
            .load(R.drawable.loading)
            .into(DrawableImageViewTarget(loadingImage))

        // 목록으로 돌아가기
        backButton.setOnClickListener{
            requireActivity().supportFragmentManager.popBackStack()
        }

        // Argument 에서 클릭된 항목의 정보를 받아와서 UI에 표시
        val gameId = arguments?.getString("gameId")
        val title = arguments?.getString("title")
        val description = arguments?.getString("description")
        val thumbnailUrl = arguments?.getString("thumbnailUrl")
        val minPlayer = arguments?.getString("minPlayer")
        val maxPlayer = arguments?.getString("maxPlayer")
        val minPlaytime = arguments?.getString("minPlaytime")
        val maxPlaytime = arguments?.getString("maxPlaytime")
        val difficulty = arguments?.getString("difficulty")!!.toInt()
        val theme = arguments?.getString("theme")

        // UI 요소에 정보 표시
        view.findViewById<TextView>(R.id.detailTitle).text = title
        view.findViewById<TextView>(R.id.detailDescription).text = description
        view.findViewById<TextView>(R.id.detailPlayer).text = if (minPlayer == maxPlayer) {
            "인원 : ${maxPlayer}명"
        } else {
            "인원 : $minPlayer ~ ${maxPlayer}명"
        }
        view.findViewById<TextView>(R.id.detailPlaytime).text = if (minPlaytime == maxPlaytime) {
            "플레이타임 : ${maxPlaytime}분"
        } else {
            "플레이타임 : $minPlaytime ~ ${maxPlaytime}분"
        }
        view.findViewById<TextView>(R.id.detailDifficulty).text = "난이도 : ${"★".repeat(difficulty)}"
        view.findViewById<TextView>(R.id.detailTheme).text = "장르 : $theme"

        thumbnailUrl?.let {
            Glide.with(requireContext())
                .load(it)
                .placeholder(R.drawable.ipad) // 로딩 중에 표시할 이미지
                .error(R.drawable.chess_black) // 로딩 실패 시 표시할 이미지
                .into(binding.detailImage)
        }
        val roomLogId = sharedPreferences.getString("roomLogId", "")
        val themeOn = sharedPreferences.getBoolean("themeOn", true)

        val returnButton = view.findViewById<TextView>(R.id.returnButton)
        val startButton = view.findViewById<TextView>(R.id.startButton)
        val themeSwitch = view.findViewById<SwitchCompat>(R.id.theme_switch)

        // DB상의 테마 온오프여부
//        println(themeOn)
        themeSwitch.isChecked = themeOn

        // 테마 온오프 스위치
        themeSwitch.setOnCheckedChangeListener { _, _ ->
            sendThemeToggle(roomLogId!!)
        }


        // 시작/반납 버튼 토글
        toggleButtons(gameId.toString(), view)

        // 시작버튼
        startButton.setOnClickListener{
            lifecycleScope.launch {
                loadingImage.visibility = View.VISIBLE
                gameId?.let{startGame(it, roomLogId!!, view)}
            }
        }

        //종료버튼
        returnButton.setOnClickListener{
            lifecycleScope.launch {
                loadingImage.visibility = View.VISIBLE
                gameId?.let{returnGame(it, roomLogId!!, view)}
            }
        }
    }


    private fun startGame(gameId: String, roomLogId: String, view: View) {
        val service = BaseApi.getInstance().create((GameOrderApi::class.java))

        val call: Call<SuccessResponse> = service.orderGame(gameId, roomLogId)
//        println("$gameId, $roomId, $storeId")

        call.enqueue(object : Callback<SuccessResponse> {
            override  fun onResponse(call: Call<SuccessResponse>, response: Response<SuccessResponse>) {
//                println(response)
                loadingImage.visibility = View.GONE
                if (response.isSuccessful) {
//                    Log.d("Order", "${response.body()}")
                    val responseResult: SuccessResponse? = response.body()
                    val success: Boolean? = responseResult?.success
                    val message: String? = responseResult?.message

//                    Log.d("Order", "message: $message")

                    // 성공 여부에 따른 로직 처리
                    if (success == true) {
                        Log.d("Order", "Success: $message")
                        saveGameId(gameId, view)
                    } else {
                        // 실패 처리
                        showFailedDialog()
                        Log.d("Order", "Failed: $message")
                    }
                } else {
                    showFailedDialog()
                    Log.d("Order", "Fail to send: ${response.errorBody()}")
                }
            }
            override fun onFailure(call: Call<SuccessResponse>, t:Throwable) {
                showFailedDialog()
                Log.d("Order", "$t")
            }
        })
    }

    private fun saveGameId(gameId: String, view: View) {
        val sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val editor = sharedPreferences.edit()
        editor.putString("gameId", gameId)
//        editor.putString("isDeli", "true")
        editor.apply()
        showOrderedDialog()
        toggleButtons(gameId, view)
    }

    private fun showOrderedDialog() {
        val alertDialogBuilder = AlertDialog.Builder(requireContext())
        alertDialogBuilder.apply {
            setTitle("주문완료")
            setMessage("주문이 완료되었습니다.\n잠시만 기다려주세요")
            setPositiveButton("확인") {dialog, _ ->
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun returnGame(gameId: String, roomLogId: String, view: View) {
        val sharedPreferences = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        val service = BaseApi.getInstance().create((GameReturnApi::class.java))

        val call: Call<SuccessResponse> = service.returnGame(roomLogId)

        call.enqueue(object : Callback<SuccessResponse> {
            override  fun onResponse(call: Call<SuccessResponse>, response: Response<SuccessResponse>) {
//                println(response)
                loadingImage.visibility = View.GONE
                if (response.isSuccessful) {
//                    Log.d("Order", "${response.body()}")
                    val responseResult: SuccessResponse? = response.body()
                    val success: Boolean? = responseResult?.success
                    val message: String? = responseResult?.message

//                    Log.d("Order", "message: $message")

                    // 성공 여부에 따른 로직 처리
                    if (success == true) {
//                        Log.d("Order", "Success: $message")
                        showReturnedDialog()
                        sharedPreferences.edit().remove("gameId").apply()
                        toggleButtons(gameId, view)
                    } else {
                        // 실패 처리
                        showFailedDialog()
                        Log.d("Order", "Failed: $responseResult")
                    }
                } else {
                    showFailedDialog()
                    Log.d("Order", "Fail to send: $response")
                }
            }
            override fun onFailure(call: Call<SuccessResponse>, t:Throwable) {
                showFailedDialog()
                Log.d("Order", "$t")
            }
        })
    }

    private fun showReturnedDialog() {
        val alertDialogBuilder = AlertDialog.Builder(requireContext())
        alertDialogBuilder.apply {
            setTitle("요청완료")
            setMessage("반납요청되었습니다.\n잠시만 기다려주세요")
            setPositiveButton("확인") {dialog, _ ->
                loadingImage.visibility = View.GONE
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun showFailedDialog() {
        val alertDialogBuilder = AlertDialog.Builder(requireContext())
        alertDialogBuilder.apply {
            setTitle("요청실패")
            setMessage("네트워크 오류가 발생했습니다.\n잠시 후 다시 시도해주세요.")
            setPositiveButton("확인") {dialog, _ ->
                loadingImage.visibility = View.GONE
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun toggleButtons(gameId: String, view: View) {
        val savedGameId = requireActivity().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
            .getString("gameId", "")

        val returnButton = view.findViewById<TextView>(R.id.returnButton)
        val startButton = view.findViewById<TextView>(R.id.startButton)

        if (gameId == savedGameId) {
            returnButton.visibility = View.VISIBLE
            startButton.visibility = View.GONE
        } else {
            returnButton.visibility = View.GONE
            startButton.visibility = View.VISIBLE
        }
    }

    private fun sendThemeToggle(roomLogId: String){
        val service = BaseApi.getInstance().create((ThemeToggleApi::class.java))

        val requestBody = mapOf(
            "roomLogId" to roomLogId
        )

        service.sendThemeToggle(requestBody).enqueue(object : Callback<SuccessResponse> {
            override fun onResponse(call : Call<SuccessResponse>, response: Response<SuccessResponse>) {
                if (response.isSuccessful) {
                    Log.d("ThemeToggle", "Success : $response")
                } else {
                    Log.d("ThemeToggle", "$response")
                }
            }
            override fun onFailure(call: Call<SuccessResponse>, t:Throwable) {
                Log.d("ThemeToggle", "$t")
            }
        })
    }
}