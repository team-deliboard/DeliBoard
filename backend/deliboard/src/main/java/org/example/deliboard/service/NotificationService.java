package org.example.deliboard.service;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.GeneralResponseDto;
import org.springframework.stereotype.Service;

import com.google.firebase.messaging.FirebaseMessaging;
import com.google.firebase.messaging.FirebaseMessagingException;
import com.google.firebase.messaging.Message;
import org.springframework.transaction.annotation.Transactional;

@Service
@Transactional
@RequiredArgsConstructor
public class NotificationService {
    public GeneralResponseDto sendNotification(String token, String title, String body) {
        if (token == null || token.isEmpty()) {
            return new GeneralResponseDto(false, "Token error");
        }
        Message message = Message.builder()
                .setToken(token)
                .putData("title", title)
                .putData("body", body)
                .build();
        try {
            FirebaseMessaging.getInstance().send(message);
            return new GeneralResponseDto(true, "Notification sent successfully");
        } catch (FirebaseMessagingException e) {
            e.printStackTrace();
            return new GeneralResponseDto(false, "Failed to send notification");
        }
    }
}
