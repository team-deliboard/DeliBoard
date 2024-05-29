package org.example.deliboard.dto;

import lombok.*;

@Getter
@Setter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class MenuDto {
    private Integer id;
    private String name;
    private Integer type;
    private Integer price;
    private Boolean isAvailable;
    private Integer storeId;
}
