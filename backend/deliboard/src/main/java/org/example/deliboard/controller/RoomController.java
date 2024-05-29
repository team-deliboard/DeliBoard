package org.example.deliboard.controller;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.*;
import org.example.deliboard.model.Room;
import org.example.deliboard.model.RoomLog;
import org.example.deliboard.repository.RoomLogRepository;
import org.example.deliboard.repository.RoomRepository;
import org.example.deliboard.service.RoomService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.concurrent.CompletableFuture;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/room")
public class RoomController {

    private final RoomService roomService;
    private final RoomRepository roomRepository;
    private final RoomLogRepository roomLogRepository;

    @PostMapping("/login")
    public GeneralResponseDto getRoomToken(@RequestBody RoomTokenDto body){
        Room room = roomRepository.findRoomByNumberAndStoreId(body.getRoomNumber(), body.getStoreId()).orElse(null);
        if (room == null)
            return new GeneralResponseDto(false, "Invalid request: Room not found");
        return roomService.updateRoomToken(room, body.getFcmToken());
    }

    @GetMapping("/theme")
    public Boolean isThemeInUse(@RequestParam Integer roomLogId) {
        RoomLog roomLog = roomLogRepository.findRoomLogByIdAndIsCheckedOut(roomLogId, false).orElse(null);
        if (roomLog == null)
            return false;
        return roomLog.getUseTheme();
    }

    @PostMapping("/theme")
    public CompletableFuture<GeneralResponseDto> setRoomTheme(@RequestBody RoomLogIdDto body){
        RoomLog roomLog = roomLogRepository.findRoomLogByIdAndIsCheckedOut(body.getRoomLogId(), false).orElse(null);
        if (roomLog == null)
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, "Invalid request: Room is empty or checked out already"));
        if (roomLog.getStock() == null)
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, "Invalid request: Stock not found"));
        if (!roomLog.getUseTheme())
            return CompletableFuture.completedFuture(new GeneralResponseDto(true, "Theme is not in use. Nothing changed"));
        return roomService.setRoomTheme(roomLog, roomLog.getUseTheme());
    }

    @PutMapping("/theme")
    public CompletableFuture<GeneralResponseDto> toggleTheme(@RequestBody RoomLogIdDto body){
        RoomLog roomLog = roomLogRepository.findRoomLogByIdAndIsCheckedOut(body.getRoomLogId(), false).orElse(null);
        if (roomLog == null)
            return CompletableFuture.completedFuture(new GeneralResponseDto(false, "Invalid request: Room is empty or checked out already"));
        return roomService.toggleTheme(roomLog);
    }

    @PostMapping
    public RoomLogDto startGame(@RequestBody RoomDto roomInfo) {
        Integer storeId = roomInfo.getStoreId();
        Byte number = roomInfo.getRoomNumber();
        Room room = roomRepository.findRoomByNumberAndStoreId(number, storeId).orElse(null);
        if (room == null)
            return new RoomLogDto(false, null, "Invalid request: Room not found");
        return roomService.startGame(room);
    }

    @PutMapping
    public GeneralResponseDto endGame(@RequestBody RoomLogIdDto body) {
        RoomLog roomLog = roomLogRepository.findRoomLogById(body.getRoomLogId()).orElse(null);
        if (roomLog == null)
            return new GeneralResponseDto(false, "Invalid request: Room not found");
        return roomService.endGame(roomLog);
    }
}
