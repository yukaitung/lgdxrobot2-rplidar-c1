FROM ros:lyrical AS builder

WORKDIR /src
COPY . .

# Fix ARM64 build network error
RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.ocf.berkeley.edu/ubuntu-ports/|g' /etc/apt/sources.list.d/ubuntu.sources
RUN rosdep update
RUN apt-get update \
    && apt-get install -y \
    # Install debian packages build dependencies
    python3-bloom python3-rosdep fakeroot debhelper dh-python \ 
    dpkg wget \
    && rosdep install --from-paths lgdx_rplidar_c1_ros2 --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* 

# Complie the packages
WORKDIR /src/lgdx_rplidar_c1_ros2
RUN bloom-generate rosdebian
RUN fakeroot debian/rules binary

# Organise the output
## Debian packages
WORKDIR /src
RUN mkdir -p /debs
RUN mv *.deb /debs
#RUN mv *.ddeb /debs

# Final stage outputs /src
FROM scratch AS export
COPY --from=builder /debs /debs