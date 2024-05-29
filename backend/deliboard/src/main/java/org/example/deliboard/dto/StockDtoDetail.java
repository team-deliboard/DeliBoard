package org.example.deliboard.dto;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

@Data
@AllArgsConstructor
@NoArgsConstructor
@Builder
public class StockDtoDetail {
    private Integer id;
    private String title;
    private String description;
    private String thumbnailUrl;
    private Integer stock;
    private Integer minPlayer;
    private Integer maxPlayer;
    private Integer minPlaytime;
    private Integer maxPlaytime;
    private Float difficulty;
    private String theme;
}
