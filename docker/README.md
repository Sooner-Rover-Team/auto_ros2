# Docker

A Docker image containing everything you need to get up and running. It has the same Ubuntu and ROS 2 version as the Rover.

![Screenshot of the image running ROS 2](./.docker_screenie.avif)

## Usage

Before anything, you'll need to install Docker.

Docker lets you separate your computer from a lot of Rover dependencies. It sets up all the code automatically, so you don't have to worry about doing it yourself. Let's install it! :D

- On macOS, install the [Docker Desktop app](https://docs.docker.com/desktop/setup/install/mac-install/). If you're asked, don't pay for it. You'll need to open the Docker app **before doing anything in the container**, meaning you'll turn it on before doing Rover stuff.
- On Windows, you'll need to [get WSL 2](https://documentation.ubuntu.com/wsl/en/stable/howto/install-ubuntu-wsl2/), then install it inside of there. Restart your computer, then open the 'Ubuntu' app from your Start menu. **Now, follow the Linux instructions below using that Ubuntu terminal.**
- For Linux, [get the Docker Engine](https://docs.docker.com/desktop/setup/install/linux/), then open a terminal to start its service: `sudo systemctl enable docker.service --now`. \*\*On Linux, you don't need Docker Desktop, which is very different! It's only required on macOS.

Alright, you now have Docker! Let's grab the Rover's code and prepare for simulation:

1. Clone this repo by running: `git clone https://github.com/Sooner-Rover-Team/auto_ros2`
1. Run `uv sync` to grab dependencies
1. Activate the virtual environment: `. .venv/bin/activate`
1. Move into the Docker container's folder with: `cd docker/`
1. Build the image with `just build`
   - This might take a bit. Grab a drink or maybe [watch some YouTube...](https://www.youtube.com/watch?v=HI_CKK-GOeU)
1. Run the Docker image in a container with `just run`

Great, you've got the Docker container!

Now, connect to the container by clicking this link: [`http://localhost:5801/vnc.html?host=localhost&port=5901`](http://localhost:5801/vnc.html?host=localhost&port=5901).

If you want, you may also use [a VNC client like TurboVNC](https://github.com/TurboVNC/turbovnc/releases/latest) - it's quite a bit faster. Install it, then connect to `localhost:5901` when prompted!

The username is `soro`, and **password is `NotIan!`** :)
