## Build immagine

```bash
docker build -t erebus-webots:latest .
```

## Avvio Webots (porta 1234)

```bash
./run-wayland.sh
```


```bash
./run-wayland.sh /opt/erebus/game/worlds/mapping_example_1.wbt 1235
```

## Shell nel container

```bash
docker run --rm -it \
  --user "$(id -u):$(id -g)" \
  -p 1234:1234 \
  --env WEBOTS_HOME=/usr/local/webots \
  --env LANG=en_US.UTF-8 \
  --env LC_ALL=en_US.UTF-8 \
  --volume "$(pwd)/..:/workspace" \
  --device /dev/dri:/dev/dri \
  --entrypoint /bin/bash \
  erebus-webots:latest
```
