#! /bin/bash

ps a | grep -E "(ros|interceptty)" | cut -f 3 -d " " | xargs -d "\n" sudo kill -9