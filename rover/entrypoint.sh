#!/bin/bash
if [ ! -d /src/install ]; then make build; fi
if [ -f /src/.autostart ]; then tmuxp load -d /src/tmuxp.yaml; fi
trap : TERM INT; sleep infinity & wait
