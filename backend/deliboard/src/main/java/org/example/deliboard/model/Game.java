package org.example.deliboard.model;

import jakarta.persistence.*;
import lombok.Data;

@Entity(name = "game")
@Data
public class Game {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Integer id;

    // 게임명
    @Column(nullable = false)
    private String title;

    // 게임 설명
    @Column
    private String detail;

    @Column
    private String theme;

    @Column
    private Integer minPlayer;

    @Column
    private Integer maxPlayer;

    @Column
    private Integer minPlaytime;

    @Column
    private Integer maxPlaytime;

    @Column
    private String category;

    @Column
    private String tag;

    @Column
    private Float difficulty;

    @Column
    private Integer bggId;
}

