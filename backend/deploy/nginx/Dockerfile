FROM nginx:1.25.4-alpine
COPY ./nginx.conf.template /etc/nginx/conf.d/default.conf.template
COPY start-nginx.sh /start-nginx.sh
RUN chmod +x /start-nginx.sh
CMD ["/start-nginx.sh"]