package org.example.deliboard.dto;

import lombok.*;

@Setter
@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class MenuOrderItemDto {
    private Integer menuId;
    private Integer quantity;
}
