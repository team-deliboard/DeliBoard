package org.example.deliboard.dto;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

@Setter
@Getter
@AllArgsConstructor
public class RoomGameDto {
    private Integer storeId;
    private Byte roomNumber;
    private Integer gameId;
}
