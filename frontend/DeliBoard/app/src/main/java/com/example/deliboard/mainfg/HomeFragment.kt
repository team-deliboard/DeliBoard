package com.example.deliboard.mainfg

import android.content.Context
import android.content.Intent
import android.content.SharedPreferences
import android.net.Uri
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.MediaController
import android.widget.VideoView
import androidx.appcompat.app.AlertDialog
import androidx.constraintlayout.widget.ConstraintLayout
import androidx.fragment.app.Fragment
import androidx.navigation.findNavController
import com.example.deliboard.BaseApi
import com.example.deliboard.R
import com.example.deliboard.room.StartingActivity
import com.example.deliboard.SuccessResponse
import com.example.deliboard.Tutorial
import com.example.deliboard.login.StartApi
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response

class HomeFragment : Fragment() {

    private lateinit var sharedPreferences: SharedPreferences

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
    }

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        // Inflate the layout for this fragment
        val view = inflater.inflate(R.layout.fragment_home, container, false)

        // 영상 틀어주는거
        val videoView: VideoView = view.findViewById(R.id.homeVideo)
        val mediaController = MediaController(requireContext())
        mediaController.setAnchorView(videoView)
        videoView.setMediaController(mediaController)

        val videoPath = "android.resource://" +requireContext().packageName + "/" + R.raw.scrabble
        val videoUri = Uri.parse(videoPath)
        videoView.setVideoURI(videoUri)
        videoView.setOnPreparedListener{mp -> mp.isLooping = true}

        videoView.start()

        view.findViewById<ConstraintLayout>(R.id.button1).setOnClickListener{
            it.findNavController().navigate(R.id.action_Home_to_GameList)
        }

//        view.findViewById<ConstraintLayout>(R.id.button2).setOnClickListener{
//            it.findNavController().navigate(R.id.action_Home_to_recommend)
//        }
//
//        view.findViewById<ConstraintLayout>(R.id.videoSpace).setOnClickListener{
//
//        }

        view.findViewById<ConstraintLayout>(R.id.homeTutorialButton).setOnClickListener{
            val intent = Intent(requireContext(), Tutorial::class.java)
            startActivity(intent)
        }
        view.findViewById<ConstraintLayout>(R.id.toBeverage).setOnClickListener{
            it.findNavController().navigate(R.id.action_Home_to_beverage)
        }
        view.findViewById<ConstraintLayout>(R.id.button6).setOnClickListener{
            it.findNavController().navigate(R.id.action_Home_to_orderHistory)
        }
        view.findViewById<ConstraintLayout>(R.id.button7).setOnClickListener{
            showOverDialog()
        }
        return view
    }

    private fun showOverDialog() {
        val alertDialogBuilder = AlertDialog.Builder(requireActivity())
        alertDialogBuilder.apply {
            setTitle("사용종료")
            setMessage("사용을 종료하시겠습니까?")
            setPositiveButton("종료") {_, _ ->
                overConfirm()
            }
            setNegativeButton("취소") {dialog, _ ->
                dialog.dismiss()
            }
        }
        val alertDialog = alertDialogBuilder.create()
        alertDialog.show()
    }

    private fun overConfirm() {
        val client = BaseApi.getInstance().create(StartApi::class.java)

        val sharedPreferences = requireContext().getSharedPreferences("RoomInfo",
            Context.MODE_PRIVATE
        )
        val pref = requireContext().getSharedPreferences("RoomInfo", Context.MODE_PRIVATE)
        sharedPreferences.edit().remove("isOccupied").apply()

        val roomLogId = pref.getString("roomLogId", null)

        val requestBody = mapOf(
            "roomLogId" to roomLogId
        )

        client.deleteRoomInfo(requestBody).enqueue(object : Callback<SuccessResponse> {
            override fun onResponse(call : Call<SuccessResponse>, response: Response<SuccessResponse>) {
                if (response.isSuccessful) {
                    val responseBody = response.body()
                    if (responseBody != null && responseBody.success) {

                        sharedPreferences.edit().remove("isOccupied").apply()

                        val intent = Intent(requireActivity(), StartingActivity::class.java)
                        intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP or Intent.FLAG_ACTIVITY_NEW_TASK)
                        startActivity(intent)
                        requireActivity().finish()
                        Log.d("Login", "login success")
                    } else {
                        Log.d("Login", "login failed $responseBody")
                        showFailure(requireContext(), "매장 번호 또는 테이블 번호가 유효하지 않습니다.")
                    }
                } else {
                    Log.d("Login", "request failed $response")
                    showFailure(requireContext(), "네트워크 오류로 실패했습니다.")
                }
            }

            override fun onFailure(call: Call<SuccessResponse>, t: Throwable) {
                Log.e("Login", "$t")
                showFailure(requireContext(), "요청에 실패했습니다.")
            }
        })
    }

    private fun showFailure(context: Context, message: String) {
        val builder = AlertDialog.Builder(context)

        builder.setTitle("인증 실패")
        builder.setMessage(message)

        builder.setPositiveButton("확인") {dialog, _ ->
            dialog.dismiss()
        }

        val dialog = builder.create()
        dialog.show()
    }
}