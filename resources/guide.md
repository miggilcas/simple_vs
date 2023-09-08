# Guide transmision using Mediametix server

## Server mediametix in drone and GCS

The second best option
docker run --rm -it --network=host -v $PWD/mediamtx.yml:/mediamtx.yml bluenviron/mediamtx

## Server mediametix in onli in GCS

the best option lest recourses from drone
Required more develop, and get warnings, and make robust

## Server mediametix only in drone

This option is no scalable because only work check the video in local network and no public ip becuse have to create a direction of each drone and it is secureless

test good quality of video.
