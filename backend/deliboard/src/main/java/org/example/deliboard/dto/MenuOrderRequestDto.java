package org.example.deliboard.dto;

import lombok.*;

import java.util.List;

@Setter
@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class MenuOrderRequestDto {
    private Integer roomLogId;
    private List<MenuOrderItemDto> itemList;
}
