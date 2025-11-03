FROM ubuntu:24.04

ARG USERNAME=user
ARG UID=1000
ARG GID=1000
ARG PYTHON_VENV_PATH=/opt/python/venv
ARG WGET_ARGS="-q --show-progress --progress=bar:force:noscroll"

ENV KITWARE_NINJA_VERSION=1.11.1.g95dee.kitware.jobserver-1
ENV CCACHE_VERSION=4.9.1
ENV LLVM_VERSION=20
ENV PROTOC_VERSION=21.7

SHELL ["/bin/bash", "-eo", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Install APT packages
RUN <<EOF
	# Enable 32-bit package support for x86-64
	if [ "${HOSTTYPE}" = "x86_64" ]; then
		dpkg --add-architecture i386
	fi

	# Upgrade system packages
	apt-get -y update
	apt-get -y upgrade

	# Install base packages
	apt-get install --no-install-recommends -y \
		software-properties-common \
		lsb-release \
		autoconf \
		automake \
		bison \
		build-essential \
		ca-certificates \
		ccache \
		chrpath \
		cmake \
		cpio \
		device-tree-compiler \
		dfu-util \
		diffstat \
		dos2unix \
		file \
		flex \
		g++ \
		gawk \
		gcc \
		gcovr \
		gdb \
		git \
		git-core \
		gnupg \
		gperf \
		help2man \
		iproute2 \
		lcov \
		libcairo2-dev \
		libglib2.0-dev \
		libgtk2.0-0 \
		liblocale-gettext-perl \
		libncurses5-dev \
		libpcap-dev \
		libpopt0 \
		libsdl1.2-dev \
		libsdl2-dev \
		libssl-dev \
		libtool \
		libtool-bin \
		locales \
		make \
		net-tools \
		ninja-build \
		nodejs \
		npm \
		openssh-client \
		parallel \
		pkg-config \
		python3 \
		python3-dev \
		python3-pip \
		python3-ply \
		python3-setuptools \
		python3-venv \
		python-is-python3 \
		rsync \
		socat \
		srecord \
		sudo \
		texinfo \
		unzip \
		valgrind \
		wget \
		ovmf \
		xz-utils \
		thrift-compiler

	# Install x86-64 image-specific packages
	if [ "${HOSTTYPE}" = "x86_64" ]; then
		# Mark `python3` as held because APT otherwise tries to remove it during i386
		# package installation.
		apt-mark hold python3 python3-apt

		# Install multi-lib gcc
		apt-get install --no-install-recommends -y \
			gcc-multilib \
			g++-multilib

		# Install 32-bit dependencies
		apt-get install --no-install-recommends -y \
			libc6-dbg:i386 \
			libfuse-dev:i386 \
			libsdl2-dev:i386
	fi
EOF

RUN <<EOF
		apt-get update

    apt-get install -y clang-20 clang-tidy-20 clang-format-20 clangd-20
        				
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-20 10
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-20 10
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-20 10
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-20 10

    apt-get autoremove --purge -y
    apt-get clean -y
    rm -rf /var/lib/apt/lists/*
EOF

# Set build environment variables
ENV PKG_CONFIG_PATH=/usr/lib/i386-linux-gnu/pkgconfig
ENV OVMF_FD_PATH=/usr/share/ovmf/OVMF.fd

# Initialise system locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# Set up Python virtual environment for Zephyr
RUN <<EOF
	# Initialise virtual environment
	mkdir -p ${PYTHON_VENV_PATH}
	python3 -m venv ${PYTHON_VENV_PATH}

	# Activate virtual environment for subsequent steps
	source ${PYTHON_VENV_PATH}/bin/activate

	# Install pip package manager
	pip install --no-cache-dir --upgrade pip setuptools wheel

	# Install Zephyr requirements
	pip install --no-cache-dir \
		-r https://raw.githubusercontent.com/zephyrproject-rtos/zephyr/main/scripts/requirements.txt \
		-r https://raw.githubusercontent.com/zephyrproject-rtos/mcuboot/main/scripts/requirements.txt \
		-r https://raw.githubusercontent.com/zephyrproject-rtos/trusted-firmware-m/main/tools/requirements.txt \
		'esptool>=5.0.2' \
		GitPython \
		junitparser \
		junit2html \
		nrf-regtool~=9.0.1 \
		numpy \
		protobuf \
		grpcio-tools \
		PyGithub \
		pylint \
		sh \
		statistics \
		west

	# Run pip check (x86 only for now because it fails on ARM)
	if [ "${HOSTTYPE}" = "x86_64" ]; then
		pip check
	fi
EOF

# Make Zephyr Python virtual environment available globally
ENV PATH=${PYTHON_VENV_PATH}/bin:$PATH

# Install Kitware ninja
# NOTE: Pre-built Kitware ninja binaries are only available for x86_64 host.
RUN <<EOF
	if [ "${HOSTTYPE}" = "x86_64" ]; then
		wget ${WGET_ARGS} https://github.com/Kitware/ninja/releases/download/v${KITWARE_NINJA_VERSION}/ninja-${KITWARE_NINJA_VERSION}_x86_64-linux-gnu.tar.gz
		tar xf ninja-${KITWARE_NINJA_VERSION}_x86_64-linux-gnu.tar.gz -C /opt
		ln -s /opt/ninja-${KITWARE_NINJA_VERSION}_x86_64-linux-gnu/ninja /usr/local/bin
		rm ninja-${KITWARE_NINJA_VERSION}_x86_64-linux-gnu.tar.gz
	fi
EOF

# Install ccache
# NOTE: Pre-built ccache binaries are only available for x86_64 host.
RUN <<EOF
	if [ "${HOSTTYPE}" = "x86_64" ]; then
		wget ${WGET_ARGS} https://github.com/ccache/ccache/releases/download/v${CCACHE_VERSION}/ccache-${CCACHE_VERSION}-linux-x86_64.tar.xz
		tar xf ccache-${CCACHE_VERSION}-linux-x86_64.tar.xz -C /opt
		ln -s /opt/ccache-${CCACHE_VERSION}-linux-x86_64/ccache /usr/local/bin
		rm ccache-${CCACHE_VERSION}-linux-x86_64.tar.xz
	fi
EOF

# Install protobuf-compiler
RUN <<EOF
	mkdir -p /opt/protoc
	cd /opt/protoc
	PROTOC_HOSTTYPE=$(case $HOSTTYPE in x86_64) echo "x86_64";; aarch64) echo "aarch_64";; esac)
	wget ${WGET_ARGS} https://github.com/protocolbuffers/protobuf/releases/download/v${PROTOC_VERSION}/protoc-${PROTOC_VERSION}-linux-${PROTOC_HOSTTYPE}.zip
	unzip protoc-${PROTOC_VERSION}-linux-${PROTOC_HOSTTYPE}.zip
	ln -s /opt/protoc/bin/protoc /usr/local/bin
	rm -f protoc-${PROTOC_VERSION}-linux-${PROTOC_HOSTTYPE}.zip
EOF

# Set up Rust
RUN <<EOF
	# Install Cargo package manager
	wget -q -O- "https://sh.rustup.rs" | sh -s -- -y --default-toolchain 1.86

	# Make Cargo globally available for subsequent steps
	PATH=~/.cargo/bin:$PATH

	# Install uefi-run utility
	cargo install uefi-run --root /usr

	# Install Rust target support required by Zephyr
	rustup target install riscv32i-unknown-none-elf
	rustup target install riscv64imac-unknown-none-elf
	rustup target install thumbv6m-none-eabi
	rustup target install thumbv7em-none-eabi
	rustup target install thumbv7m-none-eabi
	rustup target install thumbv8m.main-none-eabi
	rustup target install x86_64-unknown-none
EOF

# Create user account
RUN <<EOF
	# Remove 'ubuntu' user to free UID 1000
	userdel -r ubuntu || true

	# Create user group and user account
	groupadd -g $GID -o $USERNAME
	useradd -u $UID -m -g $USERNAME -G plugdev $USERNAME

	# Add user account to sudoers
	echo $USERNAME ' ALL = NOPASSWD: ALL' > /etc/sudoers.d/$USERNAME
	chmod 0440 /etc/sudoers.d/$USERNAME
EOF

# Ensure that container runs in the 'root' user context
USER root

ENV ZSDK_VERSION=0.17.4
ENV ZSDK_VERSION=$ZSDK_VERSION

# Install Zephyr SDK
RUN <<EOF
		mkdir -p /opt/toolchains
    cd /opt/toolchains


	  wget ${WGET_ARGS} https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz
		tar xf zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz
		 
	 	wget ${WGET_ARGS} https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZSDK_VERSION}/toolchain_linux-${HOSTTYPE}_arm-zephyr-eabi.tar.xz
		tar xf toolchain_linux-${HOSTTYPE}_arm-zephyr-eabi.tar.xz -C zephyr-sdk-${ZSDK_VERSION}/
		 
		rm zephyr-sdk-${ZSDK_VERSION}_linux-${HOSTTYPE}_minimal.tar.xz
		rm toolchain_linux-${HOSTTYPE}_arm-zephyr-eabi.tar.xz

EOF

# Run the Zephyr SDK setup script as 'user' in order to ensure that the
# `Zephyr-sdk` CMake package is located in the package registry under the
# user's home directory.
USER $USERNAME

RUN <<EOF
	sudo -E -- bash -c '
	/opt/toolchains/zephyr-sdk-${ZSDK_VERSION}/setup.sh -c &&
	chown -R $USERNAME:$USERNAME /home/$USERNAME/.cmake
	'
EOF

#########################################################
# Above is based on the zephyrprojectrtos/ci dockerfile #
#########################################################

USER root

RUN chown -R user:user /home/user

RUN apt-get update -y && apt-get install -y \
    cppcheck \
    minicom \
    curl \
    jq \
    gh \
    libusb-1.0-0

COPY resources/JLink_Linux_V854_x86_64.deb .
RUN touch /usr/bin/udevadm && chmod +x /usr/bin/udevadm
RUN sudo apt-get install -y ./JLink_Linux_V854_x86_64.deb

RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /bin

USER user

ENV DISPLAY=:0
ENV WEST_WORKSPACE=/home/user/west-workspace
ENV ZEPHYR_BASE=$WEST_WORKSPACE/zephyr
ENV APP_ROOT=$WEST_WORKSPACE/argos-smd-driver-zephyr
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
ENV PATH="${PATH}:${ZEPHYR_BASE}/scripts"
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr

COPY resources/nrfutil /usr/bin/
RUN sudo chmod 777 /usr/bin/nrfutil && \
    nrfutil self-upgrade && \
    nrfutil install device

RUN bash -c "$(wget https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh -O -)"

RUN mkdir -p $APP_ROOT
WORKDIR $APP_ROOT

COPY west.yml Makefile ./
RUN west init -l .
RUN	west update

USER root

CMD ["/bin/bash"]
