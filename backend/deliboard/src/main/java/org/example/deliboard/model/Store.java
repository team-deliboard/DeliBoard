package org.example.deliboard.model;

import jakarta.persistence.*;
import lombok.*;

@Entity(name = "store")
@Data
public class Store {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(nullable = false)
    private Integer id;

    @Column
    // 매장명
    private String name;

    @Column
    // 맵 데이터 파일 디렉토리
    private String map;

    @Column
    // 카운터 위치값(좌표)
    private String home;
}
