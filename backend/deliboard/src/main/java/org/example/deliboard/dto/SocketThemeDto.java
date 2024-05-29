package org.example.deliboard.dto;

import lombok.*;

@Setter
@Getter
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class SocketThemeDto {
    private Integer command;
    private Integer storeId;
    private Byte roomNumber;
    private String theme;
}
