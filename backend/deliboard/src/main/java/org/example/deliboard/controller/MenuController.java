package org.example.deliboard.controller;

import lombok.RequiredArgsConstructor;
import org.example.deliboard.dto.MenuDto;
import org.example.deliboard.model.Store;
import org.example.deliboard.repository.StoreRepository;
import org.example.deliboard.service.MenuService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import java.util.Collections;
import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/menu")
public class MenuController {
    private final MenuService menuService;
    private final StoreRepository storeRepository;

    @GetMapping("/list")
    public List<MenuDto> getMenuList(@RequestParam Integer storeId) {
        Store store = storeRepository.findStoreById(storeId).orElse(null);
        if (store == null)
            return Collections.emptyList();
        return menuService.getMenuList(store);
    }
}
