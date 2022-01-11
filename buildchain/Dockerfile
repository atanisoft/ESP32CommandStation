FROM espressif/idf:release-v4.4

# Add group. We chose GID 1000 as default.
RUN groupadd -g 1000 espressif

# Add user. We chose UID 1000 as default
RUN useradd espressif -d /home/espressif -u 1000 -g 1000 -m -s /bin/bash

# Copy codebasde
RUN mkdir /home/espressif/ESP32CommandStation
COPY . /home/espressif/ESP32CommandStation/
RUN chown -R espressif:espressif /home/espressif/ESP32CommandStation

# Or, clone it & checkout a specific version
#USER espressif
#RUN git clone https://github.com/atanisoft/ESP32CommandStation
#RUN cd ESP32CommandStation && git pull && git checkout v1.5.0-beta1
#USER root

# Copy build script & make executable
COPY buildchain/make_build.sh /home/espressif/make_build.sh
RUN chmod 755 /home/espressif/make_build.sh

# Move to espressif user and home
USER espressif
WORKDIR /home/espressif