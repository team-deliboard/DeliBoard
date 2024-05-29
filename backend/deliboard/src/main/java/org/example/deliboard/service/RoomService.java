package org.example.deliboard.service;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.GeneralResponseDto;
import org.example.deliboard.dto.RoomLogDto;
import org.example.deliboard.model.Room;
import org.example.deliboard.model.RoomLog;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;

import org.example.deliboard.repository.*;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.concurrent.CompletableFuture;

@Service
@Transactional
@RequiredArgsConstructor
public class RoomService {
    private final RoomRepository roomRepository;
    private final GameRepository gameRepository;
    private final RoomLogRepository roomLogRepository;

    private final SocketClientService socketClientService;

    public GeneralResponseDto updateRoomToken(Room room, String fcmToken){
        roomRepository.updateRoomToken(room.getId(), fcmToken);
        return new GeneralResponseDto(true, "Logged in: Token updated");
    }

    @Async
    public CompletableFuture<GeneralResponseDto> setRoomTheme(RoomLog roomLog, Boolean useTheme){
        socketClientService.setRoomTheme(roomLog, useTheme);
        return CompletableFuture.completedFuture(new GeneralResponseDto(true, "Room theme updated"));
    }

    @Async
    public CompletableFuture<GeneralResponseDto> toggleTheme(RoomLog roomLog){
//        Byte roomNumber = roomLog.getRoom().getNumber();
//        Integer storeId = roomLog.getRoom().getStore().getId();
//        Room room = roomRepository.findRoomByNumberAndStoreId(roomNumber, storeId).orElse(null);
        roomLog.setUseTheme(!roomLog.getUseTheme());
        roomLogRepository.save(roomLog);
        if (roomLog.getUseTheme()) {
            if (roomLog.getStock() != null)
                setRoomTheme(roomLog, true);
            return CompletableFuture.completedFuture(new GeneralResponseDto(true, "Theme turned on"));
        }
        if (roomLog.getStock() != null)
            setRoomTheme(roomLog, false);
        return CompletableFuture.completedFuture(new GeneralResponseDto(true, "Theme turned off"));
    }

    public RoomLogDto startGame(Room room){
        RoomLog roomLog = roomLogRepository.findRoomLogByIdAndIsCheckedOut(room.getId(), false).orElse(null);
        if (roomLog != null) {
            return new RoomLogDto(false, null, "Game already started");
        }
        RoomLog newRoomLog = new RoomLog();
        newRoomLog.setRoom(room);
        roomLogRepository.save(newRoomLog);
        return new RoomLogDto(true, newRoomLog.getId(), "Started");
    }

    public GeneralResponseDto endGame(RoomLog roomLog){
        roomLog.setIsCheckedOut(true);
        roomLog.setEndDttm(LocalDateTime.now());
        roomLogRepository.save(roomLog);
        return new GeneralResponseDto(true, "Ended");
    }
}
