# we'll use a base image for ros 2 humble desktop!
#
# note: this comes without the simulator, so we'll install that separately...
FROM osrf/ros:humble-simulation

################
# environment variables
################
#
# ...keep aptitude quiet
ENV DEBIAN_FRONTEND=NONINTERACTIVE
#
# ...vnc stuff
ENV VNC_PORT=5901
ENV NOVNC_PORT=5801
ENV SSHD_PORT=22
#
# ...user/password
ENV USER=soro
ENV PASS=NotIan!

################
# setup stuff
################

# install the shell
RUN apt-get update && apt-get install -y --no-install-recommends fish curl && rm -rf /var/lib/apt/lists/*

# default user
#
# ...make the user
RUN useradd \
    $USER \
    --shell /usr/bin/fish \
    --create-home \
    --groups sudo
#
# ...assign user a _very good_ password
RUN echo "$USER:$PASS" | chpasswd
#
# ...make them a `~/bin`
RUN mkdir -p /home/$USER/bin
#
# ...give them ownership of their home dir
RUN chown -R $USER /home/$USER/

# grab `bass` for `fish`
RUN su - $USER -c "fish -c 'curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && \
    fisher install jorgebucaran/fisher && \
    fisher install edc/bass && \
    fish_add_path /home/$USER/bin'"

# install the simulator
RUN apt-get update && apt-get install -y ros-humble-ros-ign-bridge ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz ros-humble-ros-gz gazebo ros-humble-geographic-msgs ros-humble-geometry-msgs && rm -rf /var/lib/apt/lists/*

# grab rust and uv
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs
COPY --from=ghcr.io/astral-sh/uv:0.6.3 /uv /uvx /bin/

# grab turbovnc, a crazy-fast vnc server
#
# ...signing key
RUN curl -Ls https://packagecloud.io/dcommander/turbovnc/gpgkey | \
    gpg --dearmor > /etc/apt/trusted.gpg.d/TurboVNC.gpg

#
# ...repo
COPY sysroot/etc/apt/sources.list.d/TurboVNC.list /etc/apt/sources.list.d/TurboVNC.list

# install all the gui stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    nginx supervisor websockify novnc \
    turbovnc xserver-xorg x11-xserver-utils \
    dbus-x11 x11-utils alsa-utils pulseaudio mesa-utils libgl1-mesa-dri \
    xorg xserver-xorg-video-dummy \
    xfce4 xfwm4 xfce4-panel xfce4-settings xfce4-clipman xfce4-session \
    thunar mousepad xfce4-terminal xfce4-taskmanager \
    wget htop ripgrep \
    sudo net-tools zenity xz-utils xinput xterm \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

# theming
RUN apt-get update \
    && apt-get install -y --no-install-recommends xfce4-whiskermenu-plugin adwaita-icon-theme-full fonts-inter gtk2-engines adwaita-icon-theme \
    && rm -rf /var/lib/apt/lists/*

# also grab alacritty, a fast (and kinda mid) terminal
RUN apt-get update && apt-get install -y --no-install-recommends software-properties-common \
    && add-apt-repository ppa:aslatter/ppa -y \
    && apt-get update \
    && apt-get install -y --no-install-recommends alacritty \
    && rm -rf /var/lib/apt/lists/*

# other userspace stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    && rm -rf /var/lib/apt/lists/*

################
# finishing touches
################

# prep for sshd
RUN mkdir /var/run/sshd

# by default, use bash for system stuff
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# sysroot
COPY sysroot/ /

# make a password file for vnc
RUN mkdir -p /home/$USER/.vnc
RUN touch /home/$USER/.vnc/passwd && chmod 600 /home/$USER/.vnc/passwd
RUN echo "$PASS" | /opt/TurboVNC/bin/vncpasswd -f >> /home/$USER/.vnc/passwd
RUN chown -R $USER /home/$USER/

# download + copy the theme over
RUN curl --location --output /tmp/adw-gtk3.tar.xz \
    https://github.com/lassekongo83/adw-gtk3/releases/download/v5.6/adw-gtk3v5.6.tar.xz \
    && xz --decompress /tmp/adw-gtk3.tar.xz \
    && tar -xf /tmp/adw-gtk3.tar --directory=/home/$USER/.local/share/themes/ \
    && rm -rf /tmp/adw-gtk3.tar

# do the same for the xfwm4 theme
RUN git clone --depth 1 --quiet \
    https://github.com/onkoe/adw-xfwm4.git \
    /tmp/adw-xfwm4 \
    && mv /tmp/adw-xfwm4/themes/adw-gtk3-dark/xfwm4 /home/soro/.local/share/themes/adw-gtk3-dark/ \
    && mv /tmp/adw-xfwm4/themes/adw-gtk3/xfwm4 /home/soro/.local/share/themes/adw-gtk3/ \
    && rm -rfd /tmp/adw-xfwm4

# install the `iosevka` font (not packaged in ubuntu 22.04)
RUN wget -O /tmp/fonts-iosevka.deb \
    http://phd-sid.ethz.ch/debian/fonts-iosevka/fonts-iosevka_22.0.0%2Bds-1_all.deb \
    && sudo dpkg -i /tmp/fonts-iosevka.deb \
    && rm /tmp/fonts-iosevka.deb \
    && sudo fc-cache -fv

# expose ports to host
EXPOSE $VNC_PORT
EXPOSE $NOVNC_PORT
EXPOSE $SSHD_PORT

# immediately run supervisord
CMD ["/usr/bin/env", "supervisord", "-c", "/etc/supervisor/conf.d/supervisord.conf", "--logfile", "/tmp/supervisord.log"]
