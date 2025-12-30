# Use Ubuntu Noble (24.04) as base
FROM ubuntu:noble

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install required tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    xz-utils \
    ca-certificates \
    build-essential \
    gdb \
    cmake \
    openocd \
    tree \
    python3 \
    python3-venv \
    python3-pip \
    doxygen \
    graphviz \
 && rm -rf /var/lib/apt/lists/*

# Set default working directory
WORKDIR /workspaces

# Create a virtual environment
RUN python3 -m venv /opt/python_dev_env

# Ensure venv is used by default
ENV PATH="/opt/python_dev_env/bin:${PATH}"

# Install protobuf libraries
RUN pip install --no-cache-dir protobuf grpcio-tools 

# Install cubemx_tool libraries
RUN pip install pyyaml

# Install telemetrix libraries
RUN pip install pyserial pyqt6

# Install power supply control library
RUN pip install spd3303x

# Install power supply control library
RUN pip install spd3303x

# Install arm gcc from official toolchain
RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz \
    && tar -xJf arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz -C /opt \
    && rm arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz

# Add to PATH
ENV PATH="/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin:${PATH}"

# Fix missing libraries expected by ARM toolchain for debugging with gdb
RUN cd /usr/lib/x86_64-linux-gnu && \
    ln -sf libncursesw.so.6 libncursesw.so.5 && \
    ln -sf libtinfo.so.6 libtinfo.so.5