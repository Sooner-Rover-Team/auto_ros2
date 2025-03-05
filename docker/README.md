# Docker

A Docker image containing everything you need to get up and running. It has the same Ubuntu and ROS 2 version as the Rover.

![Screenshot of the image running ROS 2](./.docker_screenie.avif)

## Usage

1. Install Docker and Docker Compose (you **don't** need Docker Desktop!)
1. Clone this repo
1. `uv sync`, `. .venv/bin/activate`
1. Build the image with `just build`
1. Run it with `just run`

Now, you can connect to the container by visiting this link: [`http://localhost:5801/vnc.html?host=localhost&port=5901`](http://localhost:5801/vnc.html?host=localhost&port=5901). If you want, you may also use a VNC client like TurboVNC. Just connect to `localhost:5901`!

The username is `soro`, and **password is `NotIan!`** :)
