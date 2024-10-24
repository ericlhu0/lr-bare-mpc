#!/bin/bash
python -m black . --exclude venv
docformatter -i -r . --exclude venv
isort --skip venv .
