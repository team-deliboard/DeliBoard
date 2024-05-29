package org.example.deliboard.dto;

import lombok.*;

@Getter
@Setter
@Builder
@AllArgsConstructor
@NoArgsConstructor
@ToString
public class SocketGameOrderDto {
    private Integer command;
    private Integer storeId;
    private Byte roomNumber;
    private String locationX;
    private String locationY;
}
