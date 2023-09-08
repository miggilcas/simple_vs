# Guide transmision using Mediametix server

para no gastar mucho tiempo puedo hacer consultas cada ciero tiempo del servicio , o hacer que se suscriba a un topoc que envia la gcs y con eso reiniciar
es importante volver a ver lo de la retransmision de los servidores.

```
docker run --rm -it --network=host bluenviron/mediamtx:latest
```

## Server mediametix in drone and GCS

The second best option, problem is the container dont have ffmpg commando

```
docker run --rm -it --network=host -v $PWD/mediamtx.yml:/mediamtx.yml bluenviron/mediamtx
```

## Server mediametix in onli in GCS

the best option lest recourses from drone
Required more develop, and get warnings, and make robust

## Server mediametix only in drone

This option is no scalable because only work check the video in local network and no public ip becuse have to create a direction of each drone and it is secureless

test good quality of video.
