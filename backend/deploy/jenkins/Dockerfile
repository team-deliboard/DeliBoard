FROM jenkins/jenkins:2.448
USER root
ENV TZ Asia/Seoul
COPY ./deliboard_fb_key.json /var/jenkins_home/deliboard_fb_key.json
ARG DOCKER_GID=998
RUN groupadd -g ${DOCKER_GID} docker
RUN usermod -aG docker jenkins
RUN apt-get update
RUN apt-get install -y apt-transport-https \
                       ca-certificates \
                       curl \
                       gnupg2 \
                       software-properties-common \
                       vim \
                       openssh-client
RUN curl -fsSL https://download.docker.com/linux/debian/gpg | apt-key add - && \
    add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/debian $(lsb_release -cs) stable"
RUN apt-get install -y docker-ce \
                       docker-ce-cli \
                       containerd.io
USER jenkins
