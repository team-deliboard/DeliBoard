package org.example.deliboard.dto;

import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;
import java.util.List;

@Getter
@Setter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class MenuOrderDtoCounter {
    private Long id;
    private String menuName;
    private Integer quantity;
    private LocalDateTime createdDttm;
    private Integer roomLogId;
    private Boolean isDelivered;
//    private List<MenuOrderItemDto> itemList;
}
