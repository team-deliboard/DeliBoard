//package org.example.deliboard.exception;
//
//import com.google.rpc.BadRequest;
//import org.springframework.http.HttpStatus;
//import org.springframework.http.ResponseEntity;
//import org.springframework.http.converter.HttpMessageNotReadableException;
//import org.springframework.web.bind.annotation.ExceptionHandler;
//import org.springframework.web.bind.annotation.RestControllerAdvice;
//
//@RestControllerAdvice
//public class GlobalExceptionHandler {
//    @ExceptionHandler(HttpMessageNotReadableException.class)
//    protected ResponseEntity<Object> handleHttpMessageNotReadableException(HttpMessageNotReadableException e) {
//        String message = "Invalid request body";
//        return new ResponseEntity<>(message, HttpStatus.BAD_REQUEST);
//    }
//}
