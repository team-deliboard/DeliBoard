package org.example.deliboard.dto;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

@Setter
@Getter
@AllArgsConstructor
public class RoomLogDto {
    private Boolean success;
    private Integer roomLogId;
    private String message;
}
