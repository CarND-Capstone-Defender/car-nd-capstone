### Usage of Docker

#### Initial build of Docker image (complete build)
Usage: in case of adding new libraries (e.g. upgrade of Tensorflow)

call following sequence in the shell
`./builddocker.sh`
`./rundocker.sh`

call in Docker environment
`./runme.sh`


#### Incremental build of Docker image
Usage: in case of changes in Capstone sources (Python, C++, new files,...)

call in Docker environment
`./runme.sh`

optional: in case you need a new console attached to the same docker container just open a new console and type
`./attachToContainer.sh`
