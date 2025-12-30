# Use Ubuntu Noble (24.04) as base
FROM ubuntu:noble

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install required tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    make \
    cmake \
    autoconf \
    automake \
    libtool \
    flex \
    bison \
    gdb \
    valgrind \
    doxygen \
    graphviz \
    wget \
    curl \
    ca-certificates \
    tree \
    vim \
    nano \
    tar \
    gzip \
    bzip2 \
    xz-utils \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
 && rm -rf /var/lib/apt/lists/*

# Create virtual environment for Python packages
RUN python3 -m venv /opt/python_ekf_env

# Activate venv by default
ENV PATH="/opt/python_ekf_env/bin:${PATH}"

# Install Python packages for verification and analysis
RUN pip install --no-cache-dir \
    numpy \
    scipy \
    matplotlib \
    pandas \
    cocotb \
    cocotb-test \
    pytest \
    pyyaml \
    pyserial

# Verilator is an open-source, fast SystemVerilog simulator
RUN apt-get update && apt-get install -y --no-install-recommends \
    verilator \
    libgoogle-perftools-dev \
    numactl \
    perl-doc \
 && rm -rf /var/lib/apt/lists/*

# Verify Verilator installation
RUN verilator --version

# iverilog is a lightweight Verilog simulator for quick tests
RUN apt-get update && apt-get install -y --no-install-recommends \
    iverilog \
 && rm -rf /var/lib/apt/lists/*
 
# GTKWave for viewing VCD waveform dumps
RUN apt-get update && apt-get install -y --no-install-recommends \
    gtkwave \
 && rm -rf /var/lib/apt/lists/*

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     yosys \
#  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspaces

