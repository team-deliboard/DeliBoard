package org.example.deliboard.service;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.model.Store;
import org.example.deliboard.repository.MenuRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.example.deliboard.dto.MenuDto;
import org.example.deliboard.model.Menu;
import org.springframework.transaction.annotation.Transactional;

import java.util.ArrayList;
import java.util.List;

@Service
@Transactional
@RequiredArgsConstructor
public class MenuService {
    private final MenuRepository menuRepository;

    public MenuDto convertToDto(Menu menu){
        return MenuDto.builder()
                .id(menu.getId())
                .type(menu.getType())
                .name(menu.getName())
                .price(menu.getPrice())
                .storeId(menu.getStore().getId())
                .isAvailable(menu.getIsAvailable())
                .build();
    }

    public List<MenuDto> getMenuList(Store store) {
        List<Menu> buf = menuRepository.findMenuByStoreId(store.getId());
        List<MenuDto> response = new ArrayList<>();
        for (Menu menu : buf) {
            response.add(convertToDto(menu));
        }
        return response;
    }
}
