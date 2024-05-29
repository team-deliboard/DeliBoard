package org.example.deliboard.service;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.MenuOrderDtoRoom;
import org.example.deliboard.dto.MenuOrderDtoCounter;
import org.example.deliboard.dto.MenuOrderItemDto;
import org.example.deliboard.dto.GeneralResponseDto;
import org.example.deliboard.model.*;
import org.example.deliboard.repository.MenuOrderRepository;
import org.example.deliboard.repository.MenuRepository;
import org.example.deliboard.repository.RoomRepository;
import org.example.deliboard.repository.StoreRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@Service
@Transactional
@RequiredArgsConstructor
public class MenuOrderService {
    private final MenuOrderRepository menuOrderRepository;
    private final MenuRepository menuRepository;
    private final StoreRepository storeRepository;
    private final RoomRepository roomRepository;
    private final NotificationService notificationService;

    public MenuOrderDtoCounter convertToDto(MenuOrder menuOrder){
        return MenuOrderDtoCounter.builder()
                .id(menuOrder.getId())
                .menuName(menuOrder.getMenu().getName())
                .quantity(menuOrder.getQuantity())
                .createdDttm(menuOrder.getCreatedDttm())
                .roomLogId(menuOrder.getRoomLog().getId())
                .isDelivered(menuOrder.getIsDelivered())
                .build();
    }

    public List<MenuOrderDtoCounter> getOrdersByStore(Store store){
        List<MenuOrder> buf = menuOrderRepository.findMenuOrderByIsDeliveredAndStoreId(false, store.getId());
        List<MenuOrderDtoCounter> response = new ArrayList<>();
        for (MenuOrder menuOrder : buf) {
            response.add(convertToDto(menuOrder));
        }
        return response;
    }

    public GeneralResponseDto orderMenu(RoomLog roomLog, List<MenuOrderItemDto> itemList){
        Store store = roomLog.getRoom().getStore();
//        Room counter = roomRepository.findRoomByNumberAndStoreId((byte) 0, store.getId()).orElse(null);
//        if (counter == null)
//            return new GeneralResponseDto(false, "Invalid request: Counter not found");
        List<MenuOrder> buf = new ArrayList<>();
        for (MenuOrderItemDto item : itemList) {
            Menu menu = menuRepository.findMenuById(item.getMenuId()).orElse(null);
            if (menu == null)
                return new GeneralResponseDto(false, "Invalid request: Cannot find menu with given id");
            MenuOrder newOrder = new MenuOrder();
            newOrder.setStore(store);
            newOrder.setMenu(menu);
            newOrder.setQuantity(item.getQuantity());
            newOrder.setRoom(roomLog.getRoom());
            newOrder.setRoomLog(roomLog);
            buf.add(newOrder);
        }
//        GeneralResponseDto response = notificationService.sendNotification(counter.getFcmToken(), "New order", "New order has been placed. Please check the order list.");
//        if (response.isSuccess())
        menuOrderRepository.saveAll(buf);
        return new GeneralResponseDto(true, "Order placed successfully");
//        return response;
    }

    public List<MenuOrderDtoRoom> getRoomOrders(RoomLog roomLog){
        List<MenuOrder> buf = menuOrderRepository.findMenuOrderByRoomLogId(roomLog.getId());
        List<MenuOrderDtoRoom> response = new ArrayList<>();
        for (MenuOrder menuOrder : buf) {
            response.add(new MenuOrderDtoRoom(menuOrder.getMenu().getName(), menuOrder.getQuantity()));
        }
        return response;
    }

    public GeneralResponseDto deliverMenu(MenuOrder menuOrder){
        menuOrder.setIsDelivered(true);
        menuOrder.setDeliveredDttm(LocalDateTime.now());
        menuOrderRepository.save(menuOrder);
        return new GeneralResponseDto(true, "Menu delivered successfully");
    }
}
