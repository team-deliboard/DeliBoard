#!/bin/sh
# 환경변수 값으로 nginx.conf 파일 생성
envsubst '$SERVER_DOMAIN,$API_PORT,$JENKINS_PORT,$SSL_CERT_PATH,$SSL_KEY_PATH,$JENKINS_OPTS' < /etc/nginx/conf.d/default.conf.template > /etc/nginx/conf.d/default.conf

# Nginx 실행
exec nginx -g 'daemon off;'
