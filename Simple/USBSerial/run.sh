#!/bin/bash
echo 'you gotta get into the pio venv: source ~/.platformio/penv/bin/activate'
echo 'also make sure to create the directory /share/GIT/busware-esp32'
pio run --target clean
pio run
