#! /bin/env sh
docker exec -w /home/tii -it $(cat $(dirname "$0")/.container_id) /bin/zsh