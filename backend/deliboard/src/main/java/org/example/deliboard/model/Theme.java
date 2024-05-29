package org.example.deliboard.model;


import jakarta.persistence.*;
import lombok.*;

@Entity(name = "theme")
@Data
public class Theme {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(nullable = false)
    private Integer id;

    @Column
    private String type;

    @Column
    private String video;

}

