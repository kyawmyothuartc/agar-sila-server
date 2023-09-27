#!/bin/sh
# Run SiLA server script
#~/.local/bin/pipenv run python3 -m agar_station --server-uuid d5354852-da2e-4de4-8ec5-3f17f63f6a87 --ip-address 10.35.147.113 --insecure

~/.local/bin/pipenv run python -m agar_station --server-uuid d5354852-da2e-4de4-8ec5-3f17f63f6a87 --ip-address 0.0.0.0 --port 50059 --insecure