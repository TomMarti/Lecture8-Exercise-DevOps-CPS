# Lecture 8 exercises

Tom Marti

This repo is based on a fork of: [https://github.com/erdemuysalx/px4-sim](https://github.com/erdemuysalx/px4-sim)

## Setup

1. First step is to build the different images.

```sh
chmod +x build.sh
./build.sh --all
```

2. Start the container
```sh
docker compose up
```

3. Check for sanity

    1. Open a browser, navigate to [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html), and connect to container with password 1234
    2. Open a shell in the container (`docker exec -it px4_sitl bash`)
    3. In the container run `cd /root/PX4-Autopilot`, then `make px4_sitl gz_x500`.

    The first time you run these command it may take a few minutes. At the end, you may see a quadcopter on the VNC window.

## Aufgabe 1
