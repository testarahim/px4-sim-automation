#!/bin/bash

echo "Python dependencies kuruluyor..."
pip install -r requirements.txt

echo "Klasörler oluşturuluyor..."
mkdir -p logs results

echo "Setup tamamlandı."

