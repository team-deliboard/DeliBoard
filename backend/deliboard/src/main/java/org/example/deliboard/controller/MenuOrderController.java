package org.example.deliboard.controller;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.*;
import org.example.deliboard.model.*;
import org.example.deliboard.repository.*;
import org.example.deliboard.service.MenuOrderService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/menu-order")
public class MenuOrderController {
    private final MenuOrderService menuOrderService;
    private final RoomLogRepository roomLogRepository;
    private final RoomRepository roomRepository;
    private final StoreRepository storeRepository;
    private final MenuRepository menuRepository;
    private final MenuOrderRepository menuOrderRepository;

    @GetMapping("/orderList")
    public List<MenuOrderDtoCounter> getOrders(@RequestParam Integer storeId){
        Store store = storeRepository.findStoreById(storeId).orElse(null);
        if (store == null)
            return new ArrayList<>();
        return menuOrderService.getOrdersByStore(store);
    }
    @PostMapping
    public GeneralResponseDto orderMenu(@RequestBody MenuOrderRequestDto request){
        List<MenuOrderItemDto> itemList = request.getItemList();
        if (itemList == null || itemList.isEmpty())
            return new GeneralResponseDto(false, "Invalid request: Item list is empty");
        RoomLog roomLog = roomLogRepository.findRoomLogByIdAndIsCheckedOut(request.getRoomLogId(), false).orElse(null);
        if (roomLog == null)
            return new GeneralResponseDto(false, "Invalid request: Room is empty or checked out already");
        return menuOrderService.orderMenu(roomLog, itemList);
    }

    @PutMapping
    public GeneralResponseDto deliverMenu(@RequestBody MenuOrderDtoCounter body){
        MenuOrder menuOrder = menuOrderRepository.findMenuOrderById(body.getId()).orElse(null);
        if (menuOrder == null)
            return new GeneralResponseDto(false, "Invalid request: Menu order not found");
        return menuOrderService.deliverMenu(menuOrder);
    }

    @GetMapping("/room")
    public List<MenuOrderDtoRoom> getRoomOrders(@RequestParam Integer roomLogId) {
        RoomLog roomLog = roomLogRepository.findRoomLogById(roomLogId).orElse(null);
        if (roomLog == null)
            return Collections.emptyList();
        return menuOrderService.getRoomOrders(roomLog);
    }
}
